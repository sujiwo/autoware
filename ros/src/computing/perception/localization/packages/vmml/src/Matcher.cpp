/*
 * Matcherx.cpp
 *
 *  Created on: Jan 12, 2019
 *      Author: sujiwo
 */

#include <algorithm>
#include <Matcher.h>
#include "KeyFrame.h"

#include "utilities.h"
#include "triangulation.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

//#include <pcl/registration/ndt.h>
#include <pcl_omp_registration/ndt.h>

using namespace std;
using namespace Eigen;


typedef
	std::map<kpid, std::set<kpid>>
		WhichKpId;

float
	Matcher::circleOfConfusionDiameter = 1.0;

const cv::Scalar
	colorBlue(255, 0, 0),
	colorGreen(0, 255, 0),
	colorRed(0, 0, 255),
	colorYellow(0, 255, 255);
const int
	pointRadius = 3;

int
	Matcher::__maxDraw = 1;


int countFeaturePairMask(const cv::Mat &M)
{
	assert (M.cols==1 and M.type()==CV_8UC1);

	int s=0;
	for (int r=0; r<M.rows; ++r) {
		if (M.at<char>(r,0)!=0)
			s+=1;
	}

	return s;
}


void filterFeaturePairMask(std::vector<Matcher::KpPair> &srcFeaturePair, const cv::Mat &M)
{
	assert (M.rows==srcFeaturePair.size());
	vector<Matcher::KpPair> newPairList;

	for (int i=0; i<srcFeaturePair.size(); ++i) {
		if (M.at<char>(i,0)!=0)
			newPairList.push_back(srcFeaturePair.at(i));
	}

	srcFeaturePair = newPairList;
}


struct FrameKeyPointFeatures
{
	vector<cv::KeyPoint> keypointList;
	cv::Mat fFeatures;

	FrameKeyPointFeatures(
		const BaseFrame &frame,
		cv::Ptr<cv::FeatureDetector> fdetector,
		cv::Mat mask)
	{
		frame.computeFeatures(fdetector, keypointList, fFeatures, mask);
	}

	const cv::KeyPoint& keypoint(const int i) const
	{ return keypointList.at(i); }

	Vector2d keypointv(const int i) const
	{ return Vector2d(keypointList[i].pt.x, keypointList[i].pt.y); }
};


void
Matcher::decomposeE (
	const Eigen::Matrix3d &E,
	Eigen::Matrix3d &R1, Eigen::Matrix3d &R2,
	Eigen::Vector3d &t)
{
	JacobiSVD <Matrix3d> svd(E, ComputeFullU|ComputeFullV);
	Matrix3d U, W, V;
	U = svd.matrixU();
	V = svd.matrixV();

	t = U.col(2);
	t /= t.norm();

	W = Matrix3d::Zero();
	W(0,1) = -1;
	W(1,0) = 1;
	W(2,2) = 1;

	R1 = U * W * V.transpose();
	if (R1.determinant() < 0)
		R1 = -R1;
	R2 = U * W.transpose() * V.transpose();
	if (R2.determinant() < 0)
		R2 = -R2;
}


/*
 * Create an epipolar line in Frame 2 based on Fundamental Matrix F12, using a keypoint from Frame 1
 */
Line2 createEpipolarLine (const Matrix3d &F12, const Vector2d &kp1)
{
	Line2 epl2;
	epl2.coeffs() = F12 * kp1.homogeneous();
	epl2.normalize();
	return epl2;
}


Line2 createEpipolarLine (const Matrix3d &F12, const cv::KeyPoint &kp1)
{
	return createEpipolarLine( F12, Vector2d(kp1.pt.x, kp1.pt.y) );
}


double __d;


bool Matcher::isKeypointInEpipolarLine (const Line2 &epl2, const cv::KeyPoint &cvkp2)
{
	return isKeypointInEpipolarLine(epl2, Vector2d(cvkp2.pt.x, cvkp2.pt.y));
}


bool
Matcher::isKeypointInEpipolarLine (const Line2 &epl2, const Eigen::Vector2d &kp2)
{
	auto cof = epl2.coeffs();
	auto d = abs( cof.dot(kp2.homogeneous()) / sqrt(cof[0]*cof[0] + cof[1]*cof[1]) );
//	auto lim = 3.84*VMap::mScaleFactors[cvkp2.octave];
	auto lim = 3.84 * circleOfConfusionDiameter;

	// XXX: Using scale factor makes us more dependent to ORB
	if (d > lim)
		return false;
	else {
		__d = d;
		return true;
	}
}


/*
 * This function matches features from two images using
 * epipolar geometry
 */
void
Matcher::matchAny(
	const BaseFrame &Fr1,
	const BaseFrame &Fr2,
	std::vector<KpPair> &featurePairs,
	cv::Ptr<cv::DescriptorMatcher> matcher)
{
	featurePairs.clear();

	// Establish initial correspondences
	vector<cv::DMatch> initialMatches;
	matcher->match(Fr1.fDescriptors, Fr2.fDescriptors, initialMatches);

	// Sort by `distance'
	sort(initialMatches.begin(), initialMatches.end());

	const int MaxBestMatch = initialMatches.size();

	// Select N best matches
	vector<cv::Point2f> pointsIn1(MaxBestMatch), pointsIn2(MaxBestMatch);
	for (int i=0; i<MaxBestMatch; ++i) {
		auto &m = initialMatches[i];
		pointsIn1[i] = Fr1.fKeypoints[m.queryIdx].pt;
		pointsIn2[i] = Fr2.fKeypoints[m.trainIdx].pt;
	}
	cv::Mat mask;
	cv::Mat Fcv = cv::findFundamentalMat(pointsIn1, pointsIn2, cv::FM_RANSAC, 3.84*Matcher::circleOfConfusionDiameter, 0.99, mask);
	// Need Eigen Matrix of F

	for (int i=0; i<initialMatches.size(); ++i) {
		auto &m = initialMatches[i];
		if (mask.at<char>(i,0)!=0)
			featurePairs.push_back(make_pair(m.queryIdx, m.trainIdx));
	}
	return;

	Matrix3d F12;
	cv2eigen(Fcv, F12);

	/*
	 * Select point pairs that obey epipolar geometry
	 */
	for (int i = 0; i < initialMatches.size(); ++i) {
		auto &m = initialMatches[i];
		const Vector2d keypoint1v = Fr1.keypointv(m.queryIdx);
		const Line2 epl2 = createEpipolarLine(F12, Fr1.fKeypoints[m.queryIdx]);
		const Line2 epl1 = createEpipolarLine(F12.transpose(),
				Fr2.fKeypoints[m.trainIdx]);
		if (isKeypointInEpipolarLine(epl2, Fr2.keypointv(m.trainIdx)) == true
				and isKeypointInEpipolarLine(epl1, Fr1.keypointv(m.queryIdx))
					== true) {
			featurePairs.push_back(make_pair(m.queryIdx, m.trainIdx));
		}
	}
}


/*
 * Calculate Essential Matrix from F1 & F2, using featurePairs selected by matchAny().
 * We assume that those pairs obey epipolar geometry principle.
 */
TTransform
Matcher::calculateMovement (
	const BaseFrame &F1, const BaseFrame &F2,
	const std::vector<KpPair> &featurePairs,
	vector<KpPair> &validPairsByTriangulation)
{
	validPairsByTriangulation = featurePairs;

	vector<cv::Point2f> pointsIn1(featurePairs.size()), pointsIn2(featurePairs.size());
	for (int i=0; i<featurePairs.size(); ++i) {
		auto &m = featurePairs[i];
		pointsIn1[i] = F1.fKeypoints[m.first].pt;
		pointsIn2[i] = F2.fKeypoints[m.second].pt;
	}

	cv::Mat E12, R12e, te, mask;
	E12 = cv::findEssentialMat(pointsIn1, pointsIn2, F1.cameraParam.toCvMat(), cv::RANSAC, 0.999, 3.84*Matcher::circleOfConfusionDiameter, mask);

	int inliers;
	inliers = cv::recoverPose(E12, pointsIn1, pointsIn2, F1.cameraParam.toCvMat(), R12e, te, mask);

	filterFeaturePairMask(validPairsByTriangulation, mask);

	Matrix3d R12;
	Vector3d t;
	cv2eigen(R12e, R12);
	cv2eigen(te, t);

//	return T12;
	return TTransform::from_R_t(t, R12);
}


void
Matcher::matchMapPoints(
	const KeyFrame &KFsrc,
	const BaseFrame &Ft,
	std::vector<KpPair> &featurePairs,
	cv::Ptr<cv::DescriptorMatcher> matcher)
{
	featurePairs.clear();

	// Establish initial correspondences, with mask
	vector<cv::DMatch> initialMatches;
	cv::Mat mmask = cv::Mat::zeros(KFsrc.numOfKeyPoints(), Ft.numOfKeyPoints(), CV_8UC1);
	auto mMapPt = KFsrc.parentMap->allMapPointsAtKeyFrame(KFsrc.id);
	for (auto &p: mMapPt) {
		for (int _=0; _<mmask.cols; ++_)
			mmask.at<uchar>(p.second, _) = 0xff;
	}

	matcher->match(KFsrc.fDescriptors, Ft.fDescriptors, initialMatches, mmask);

	// Sort by `distance'
	sort(initialMatches.begin(), initialMatches.end());
	vector<int> inliersMatch;

	const int MaxBestMatch = 500;
	int howmany = std::min(MaxBestMatch, static_cast<int>(initialMatches.size()));

	// Select N best matches
	vector<cv::Point2f> pointsIn1(MaxBestMatch), pointsIn2(MaxBestMatch);
	for (int i=0; i<howmany; ++i) {
		auto &m = initialMatches[i];
		pointsIn1[i] = KFsrc.fKeypoints[m.queryIdx].pt;
		pointsIn2[i] = Ft.fKeypoints[m.trainIdx].pt;
	}
	cv::Mat Fcv = cv::findFundamentalMat(pointsIn1, pointsIn2, cv::FM_RANSAC, 3.84*Matcher::circleOfConfusionDiameter);
	// Need Eigen Matrix of F

	Matrix3d F12;
	cv2eigen(Fcv, F12);

	/*
	 * Guided Match using new F
	 */
	// Prepare constraints
	cv::Mat gdMask = cv::Mat::zeros(KFsrc.numOfKeyPoints(), Ft.numOfKeyPoints(), CV_8UC1);
	float maxDistance = -1;
	for (int i=0; i<initialMatches.size(); ++i) {
		auto &m = initialMatches[i];
		const Vector2d keypoint1v = KFsrc.keypointv(m.queryIdx);
		const Line2 epl2 = createEpipolarLine(F12, KFsrc.fKeypoints[m.queryIdx]);
		const Line2 epl1 = createEpipolarLine(F12.transpose(), Ft.fKeypoints[m.trainIdx]);
		if (isKeypointInEpipolarLine(epl2, Ft.keypointv(m.trainIdx))==true and isKeypointInEpipolarLine(epl1, KFsrc.keypointv(m.queryIdx))==true) {
			featurePairs.push_back(make_pair(m.queryIdx, m.trainIdx));
		}
	}
}


void
Matcher::solvePose(
	const KeyFrame &KFsrc,
	const BaseFrame &Ft,
	std::vector<KpPair> &featurePairs,
	cv::Ptr<cv::DescriptorMatcher> matcher,
	Pose &newFramePose)
{
	Eigen::Matrix4d eKfExt = KFsrc.externalParamMatrix4();
	Matrix3d KfRotation = eKfExt.block<3,3>(0,0);
	cv::Mat cKfRotMat, cRVec;
	cv::eigen2cv(KfRotation, cKfRotMat);
	cv::Rodrigues(cKfRotMat, cRVec);

	Eigen::Vector3d eKfTransVec = eKfExt.block<3,1>(0,3);
	cv::Mat cKfTransVec;
	cv::eigen2cv(eKfTransVec, cKfTransVec);

	cv::Mat
		objectPoints (featurePairs.size(), 3, CV_32F),
		imagePoints (featurePairs.size(), 2, CV_32F);
	for (int r=0; r<featurePairs.size(); ++r) {
		mpid mp = KFsrc.parentMap->getMapPointByKeypoint(KFsrc.id, featurePairs[r].first);
		const MapPoint &P = *KFsrc.parentMap->mappoint(mp);
		objectPoints.at<float>(r, 0) = P.X();
		objectPoints.at<float>(r, 1) = P.Y();
		objectPoints.at<float>(r, 2) = P.Z();
		imagePoints.at<float>(r, 0) = Ft.keypoint(featurePairs[r].second).pt.x;
		imagePoints.at<float>(r, 1) = Ft.keypoint(featurePairs[r].second).pt.y;
	}

	cv::Mat cameraMatrix = KFsrc.getCameraParameters().toCvMat();

	cv::Mat inlierIdx;

	bool hasSolution = cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, cv::Mat(), cRVec, cKfTransVec, true, 100, 4.0, 0.99, inlierIdx, cv::SOLVEPNP_EPNP);
	if (hasSolution==false)
		throw runtime_error("No solutions found");

	cv::Rodrigues(cRVec, cKfRotMat);
	cv::cv2eigen(cKfRotMat, KfRotation);
	Eigen::Quaterniond Q;
	Q = KfRotation;
	cv::cv2eigen(cKfTransVec, eKfTransVec);

	newFramePose = Pose::from_Pos_Quat(eKfTransVec, Q).inverse();

	vector<KpPair> inliers(inlierIdx.rows);
	for (int r=0; r<inlierIdx.rows; ++r) {
		inliers[r] = featurePairs.at(inlierIdx.at<int>(r,0));
	}

	featurePairs = inliers;
}


void
Matcher::rotationFinder
(const BaseFrame &Fr1, const BaseFrame &Fr2,
const std::vector<KpPair> &featurePairs,
double &theta, double &phi)
{
	// estimate phi using normal
	phi = acos(Fr1.normal().dot(Fr2.normal()));
	theta = phi*2;

	const int numPairs = min(500, static_cast<int>(featurePairs.size()));

	MatrixX3d PF1, PF2;
	PF1.resize(numPairs, Eigen::NoChange);
	PF2.resize(numPairs, Eigen::NoChange);
	for (int i=0; i<numPairs; ++i) {
		Vector3d P1 = Fr1.keypointn(featurePairs[i].first);
		Vector3d P2 = Fr2.keypointn(featurePairs[i].second);
		PF1.row(i) = P1;
		PF2.row(i) = P2;
	}

	VectorXd F;
	F.resize(numPairs);
	MatrixX2d J;
	J.resize(numPairs, Eigen::NoChange);

	for (int i=0; i<10; ++i) {
		// the function
		for (int j=0; j<numPairs; ++j) {
			Vector3d P1 = PF1.row(j), P2 = PF2.row(j);
			F[j] = -P1.x()*P2.y()* cos(phi) + P1.y()*P2.x()*cos(theta-phi) + P1.z()*P2.y()*sin(phi) + P1.y()*P2.z()*sin(theta-phi);

			J(j, 0) = -P1.y()*P2.x()*sin(theta-phi) + P1.y()*P2.z()*cos(theta-phi);
			J(j, 1) = P1.x()*P2.y()*sin(phi) + P1.y()*P2.x()*sin(theta-phi) - P1.y()*P2.z()*cos(theta-phi) + P1.z()*P2.y()*cos(phi);
		}

		MatrixXd Jinv = pseudoInverse(J);
		Vector2d X = Jinv * F;
		theta = theta - X[0];
		phi = phi - X[1];
	}

	return;
}


double
Matcher::getCameraBaselinkOffset
(const Pose &baselinkPose1, const Pose &baselinkPose2, const double &theta, const double &phi)
{
	double
		rho = (baselinkPose2.position()-baselinkPose1.position()).norm();
	double
		L = rho * (-sin(theta/2 - phi) / ( sin(phi) + sin(theta-phi) ));

	// XXX: Get L from somewhere else
	double
		lambda = -2 * L * sin(theta/2) / sin(theta/2 - phi);

	return L;
}



cv::Mat
Matcher::drawMatches(
	const BaseFrame &F1,
	const BaseFrame &F2,
	const std::vector<KpPair> &featurePairs,
	DrawMode mode,
	int maxNumOfPairs)
{
	cv::Mat result(std::max(F1.height(), F2.height()), F1.width()+F2.width(), F1.image.type());
	F1.image.copyTo( result(cv::Rect(0,0,F1.width(),F1.height())) );
	F2.image.copyTo( result(cv::Rect(F1.width(),0,F2.width(),F2.height())) );
//	cv::Mat result = F2.image.clone();
	if (maxNumOfPairs<0)
		maxNumOfPairs = featurePairs.size();
	else
		maxNumOfPairs = min(maxNumOfPairs, static_cast<int>(featurePairs.size()));

	vector<pair<cv::Point2f, cv::Point2f>> pointPairList(featurePairs.size());
	for (int i=0; i<featurePairs.size(); ++i) {
		cv::Point2f p2 = F2.fKeypoints[featurePairs[i].second].pt;
		p2.x += F1.width();
		pointPairList[i] = make_pair(
			F1.fKeypoints[featurePairs[i].first].pt,
			p2);
	}

	// Draw P2 in P1
	Pose P1z = F2.pose();
	Vector2d C2in1 = F1.project(P1z.position());
	cv::circle(result, cv::Point2f(C2in1.x(), C2in1.y()), pointRadius*2, colorYellow);

	if (mode==DrawOpticalFlow) {
		for (int n=0; n<maxNumOfPairs; ++n) {
			auto &pr = pointPairList[n];
			cv::circle(result, pr.first, pointRadius, colorBlue);
			cv::circle(result, pr.second, pointRadius, colorBlue);
			cv::Point2f pt1s = pr.first;
			pt1s.x += F1.width();
			cv::line(result, pt1s, pr.second, colorGreen);
		}
	}

	else if (mode==DrawSideBySide) {
		for (int n=0; n<maxNumOfPairs; ++n) {
			auto &pr = pointPairList[n];
			cv::circle(result, pr.first, pointRadius, colorBlue);
			cv::circle(result, pr.second, pointRadius, colorBlue);
			cv::line(result, pr.first, pr.second, colorGreen);
		}
	}

	else if (mode==DrawOnlyPoints) {
		for (int n=0; n<maxNumOfPairs; ++n) {
			auto &pr = pointPairList[n];
			cv::circle(result, pr.first, pointRadius, colorBlue);
			cv::circle(result, pr.second, pointRadius, colorBlue);
		}
	}

	else throw runtime_error("Invalid mode");

	return result;
}


int
Matcher::CheckRT (
	const Eigen::Matrix3d &R, const Eigen::Vector3d &t,
	const BaseFrame &F1, const BaseFrame &F2,
	const std::vector<KpPair> &featurePairs,
	std::vector<bool> &goodFeaturePairs,
	float &parallax)
{
	int nGood = 0;
	Vector3d
		origin1 = Vector3d::Zero(),
		origin2 = -R.transpose() * t;

	// Build new projection matrices
	// For camera 1: K[I | O]
	BaseFrame::ProjectionMat P1 = BaseFrame::ProjectionMat::Zero();
	P1.block<3,3>(0,0) = F1.cameraParam.toMatrix3();

	// For camera 2: K[R | t]
	BaseFrame::ProjectionMat P2 = BaseFrame::ProjectionMat::Zero();
	P2.block<3,3>(0,0) = R;
	P2.block<3,1>(0,3) = t;
	P2 = F2.cameraParam.toMatrix3() * P2;

	vector<double> vCosParallax(featurePairs.size());

	/*
	 * Triangulate each point pair as if first camera is in the origin
	 */
	for (int ip=0; ip<featurePairs.size(); ++ip) {

		Vector2d
			pt1 = F1.keypointv(featurePairs[ip].first),
			pt2 = F2.keypointv(featurePairs[ip].second);

		Vector4d point3D_;
		TriangulateDLT(P1, P2, pt1, pt2, point3D_);
		Vector3d point3D = (point3D_ / point3D_[3]).hnormalized();

		if (!isfinite(point3D[0]) or !isfinite(point3D[1]) or !isfinite(point3D[2]) )
			continue;

		Vector3d
			normal1 = point3D - origin1,
			normal2 = point3D - origin2;
		double
			dist1 = normal1.norm(),
			dist2 = normal2.norm();

		double cosParallax = normal1.dot(normal2) / (dist1 * dist2);

		// Check that depth regarding camera1&2 must be positive
		if (cosParallax < 0.99998) {
			if (point3D[2]<=0)
				continue;
			Vector3d point3D2 = R*point3D + t;
			if (point3D2[2]<=0)
				continue;
		}

		// Check reprojection errors
		Vector3d
			proj1 = P1 * point3D.homogeneous(),
			proj2 = P2 * point3D.homogeneous();
		proj1 /= proj1[2];
		proj2 /= proj2[2];

		float
			squareError1 = pow((proj1.hnormalized()-pt1).norm(), 2),
			squareError2 = pow((proj2.hnormalized()-pt2).norm(), 2),
			threshold = pow(Matcher::circleOfConfusionDiameter, 2);
		if (squareError1 > threshold or squareError2 > threshold)
			continue;

		nGood++;
		goodFeaturePairs[ip] = true;
	}

	if (nGood > 0) {
		// XXX: Unfinished, we do not collect parallaxes
	}
	else
		parallax = 0;

	return nGood;
}


// Match with homography constraints
void
Matcher::matchH(
	const BaseFrame &Fs1, const BaseFrame &Fs2,
	cv::Mat planeMask,
	cv::Ptr<cv::FeatureDetector> fdetector,
	cv::Ptr<cv::DescriptorMatcher> fmatcher,
	Eigen::Matrix3d &H)
{
	FrameKeyPointFeatures
		F1(Fs1, fdetector, planeMask),
		F2(Fs2, fdetector, planeMask);

	// Establish initial correspondences
	vector<cv::DMatch> initialMatches;
	fmatcher->match(F1.fFeatures, F2.fFeatures, initialMatches);

	// Sort by `distance'
	sort(initialMatches.begin(), initialMatches.end());

	const int MaxBestMatch = 500;
	int howmany = std::min(MaxBestMatch, static_cast<int>(initialMatches.size()));

	// Select N best matches
	vector<cv::Point2f> pointsIn1(howmany), pointsIn2(howmany);
	for (int i=0; i<howmany; ++i) {
		auto &m = initialMatches[i];
		pointsIn1[i] = F1.keypoint(m.queryIdx).pt;
		pointsIn2[i] = F2.keypoint(m.trainIdx).pt;
	}
	cv::Mat inlierMask;
	cv::Mat Hcv = cv::findHomography(pointsIn1, pointsIn2, cv::RANSAC, 3, inlierMask);
	cv2eigen(Hcv, H);

	// Check how many inliers we have
	int nGood = 0;
	for (int i=0; i<howmany; ++i) {
		if(inlierMask.at<int>(i,0) != 0)
			nGood++;
	}

	return;
}


TTransform
Matcher::matchLidarScans(const MeidaiDataItem &frame1, const MeidaiDataItem &frame2)
{
	ptime scantime_frame1, scantime_frame2;
	auto
		pcscan1 = const_cast<MeidaiDataItem&>(frame1).getLidarScan(&scantime_frame1),
		pcscan2 = const_cast<MeidaiDataItem&>(frame2).getLidarScan(&scantime_frame2);

	pcscan1 = LidarScanBag::VoxelGridFilter(pcscan1);
	pcscan2 = LidarScanBag::VoxelGridFilter(pcscan2);

//	pcl::NormalDistributionsTransform<LidarScanBag::point3_t, LidarScanBag::point3_t> ndt;
	pcl_omp::NormalDistributionsTransform<LidarScanBag::point3_t, LidarScanBag::point3_t> ndt;
	ndt.setTransformationEpsilon(0.01);
	ndt.setStepSize(0.1);
	ndt.setResolution(1.0);
	ndt.setMaximumIterations(30);
	ndt.setInputSource(pcscan1);
	ndt.setInputTarget(pcscan2);

	LidarScanBag::scan_t finalCl;
	TTransform guess12;
	ndt.align(finalCl);

	if (ndt.hasConverged()) {
		cout << "Converged; Score: " << ndt.getFitnessScore() << endl;
		guess12 = ndt.getFinalTransformation().cast<double>();
	}

	else {
		cout << "Not converged" << endl;
		return Matrix4d::Identity();
	}

	double velocity = guess12.translation().norm() / toSeconds(scantime_frame2-scantime_frame1);
	guess12.translation() = guess12.translation().normalized() * velocity * toSeconds(frame2.getTimestamp()-frame1.getTimestamp());
	return guess12;
}
