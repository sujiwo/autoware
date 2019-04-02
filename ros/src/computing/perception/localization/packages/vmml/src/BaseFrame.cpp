/*
 * BaseFrame.cpp
 *
 *  Created on: Oct 31, 2018
 *      Author: sujiwo
 */


#include <pcl/filters/frustum_culling.h>
#include <exception>
#include "BaseFrame.h"
#include "MapPoint.h"



using namespace Eigen;
using namespace std;

typedef Matrix<double,3,4> poseMatrix;
typedef Matrix4d poseMatrix4;


BaseFrame::BaseFrame()
{
	// TODO Auto-generated constructor stub
}

BaseFrame::~BaseFrame()
{
	// TODO Auto-generated destructor stub
}


void
BaseFrame::setPose (const g2o::SE3Quat &pq)
{
	auto Q = pq.rotation().inverse();
	auto P = -(Q * pq.translation());
	setPose(P, Q);
}


Eigen::Vector2d
BaseFrame::project (const Eigen::Vector3d &pt3) const
{
	Vector3d ptx = projectionMatrix() * pt3.homogeneous();
	return ptx.head(2) / ptx[2];
}


Eigen::Vector3d
BaseFrame::project3 (const Eigen::Vector3d &pt3) const
{
	return projectionMatrix() * pt3.homogeneous();
}


Eigen::Vector2d
BaseFrame::project (const MapPoint &pt3) const
{
	return project(pt3.getPosition());
}



Vector3d
BaseFrame::transform (const Eigen::Vector3d &pt3) const
{
	Vector4d P = externalParamMatrix4() * pt3.homogeneous();
	return P.hnormalized();
}


poseMatrix4
BaseFrame::externalParamMatrix4 () const
{
	return createExternalParamMatrix4(mPose);
}


Eigen::Matrix4d
BaseFrame::createExternalParamMatrix4(const Pose &ps)
{
	poseMatrix4 ex = poseMatrix4::Identity();
	Matrix3d R = ps.orientation().toRotationMatrix().transpose();
	ex.block<3,3>(0,0) = R;
	ex.col(3).head(3) = -(R*ps.position());
	return ex;
}


BaseFrame::ProjectionMat
BaseFrame::projectionMatrix () const
{
	return cameraParam.toMatrix() * externalParamMatrix4();
}


Vector3d
BaseFrame::normal() const
{
	return externalParamMatrix4().block(0,0,3,3).transpose().col(2).normalized();
}


void
BaseFrame::computeFeatures (cv::Ptr<cv::FeatureDetector> fd, const cv::Mat &mask)
{
	computeFeatures(fd, fKeypoints, fDescriptors, mask);
}


void
BaseFrame::computeFeatures (cv::Ptr<cv::FeatureDetector> fd, std::vector<cv::KeyPoint> &kpList, cv::Mat &descriptors, const cv::Mat &mask) const
{
	assert (image.empty() == false);

	// Clear all previous features, to be safe
	kpList.clear();
	descriptors.release();

	// Enforce gray image before computing features
	cv::Mat grayImg;
	if (image.channels()==1)
		grayImg = image;
	else
		cv::cvtColor(image, grayImg, CV_BGR2GRAY, 1);

	fd->detectAndCompute(
		grayImg,
		mask,
		kpList,
		descriptors,
		false);
}


void
BaseFrame::perturb (PerturbationMode mode,
	bool useRandomMotion,
	double displacement, double rotationAngle)
{
	Vector3d movement;
	switch (mode) {
	case PerturbationMode::Lateral:
		movement = Vector3d(1,0,0); break;
	case PerturbationMode::Vertical:
		movement = Vector3d(0,-1,0); break;
	case PerturbationMode::Longitudinal:
		movement = Vector3d(0,0,1); break;
	}
	movement = displacement * movement;

	mPose = mPose.shift(movement);
}


std::vector<BaseFrame::PointXYI>
BaseFrame::projectLidarScan
(pcl::PointCloud<pcl::PointXYZ>::ConstPtr lidarScan, const TTransform &lidarToCameraTransform, const CameraPinholeParams &cameraParams)
{
	std::vector<PointXYI> projections;

	// Create fake frame
	BaseFrame frame;
	frame.setPose(lidarToCameraTransform);
	frame.setCameraParam(&cameraParams);

	projections.resize(lidarScan->size());
	int i=0, j=0;
	for (auto it=lidarScan->begin(); it!=lidarScan->end(); ++it) {
		auto &pts = *it;
		Vector3d pt3d (pts.x, pts.y, pts.z);

		auto p3cam = frame.externalParamMatrix4() * pt3d.homogeneous();
		if (p3cam.z() >= 0) {
			auto p2d = frame.project(pt3d);
			projections[i] = PointXYI(p2d.x(), p2d.y(), j);
			++i;
		}
		j++;
	}

	return projections;
}


g2o::SBACam
BaseFrame::forG2O () const
{
	// XXX: Verify this
	g2o::SBACam mycam(orientation(), -orientation().toRotationMatrix()*position());
	mycam.setKcam(cameraParam.fx, cameraParam.fy, cameraParam.cx, cameraParam.cy, 0);

	return mycam;
}


Plane3
BaseFrame::projectionPlane() const
{
	if (cameraParam.fx==0)
		throw runtime_error("Camera parameter is not defined");

	Vector3d centerPt = mPose * Vector3d(0, 0, cameraParam.f());

	Plane3 P(normal(), centerPt);
	P.normalize();
	return P;
}


Eigen::Matrix3d
BaseFrame::FundamentalMatrix(const BaseFrame &F1, const BaseFrame &F2)
{
	if (F1.cameraParam.fx==0 or F2.cameraParam.fx==0)
		throw runtime_error("Camera parameters are not defined");

	TTransform T12 = F1.mPose.inverse() * F2.mPose;
	Matrix3d R = T12.rotation();
	Vector3d t = T12.translation();

	Vector3d A = F1.cameraParam.toMatrix3() * R.transpose() * t;
	Matrix3d C = Matrix3d::Zero();
	C(0,1) = -A[2];
	C(0,2) = A[1];
	C(1,0) = A[2];
	C(1,2) = -A[0];
	C(2,0) = -A[1];
	C(2,1) = A[0];

	return F2.cameraParam.toMatrix3().inverse().transpose() * R * F1.cameraParam.toMatrix3().transpose() * C;
	// XXX: Change to general version using Pseudo-inverse

/*
	Vector3d e0 = F1.project3(F1.position());

	Vector3d e2 = F2.project3(F1.position());
	Matrix3d C = Matrix3d::Zero();
	C(0,1) = -e2[2];
	C(0,2) = e2[1];
	C(1,0) = e2[2];
	C(1,2) = -e2[0];
	C(2,0) = -e2[1];
	C(2,1) = e2[0];

	auto P1inv = pseudoInverse(F1.projectionMatrix());
	Matrix3d F12 = C * F2.projectionMatrix() * P1inv;
	return F12;
*/
}


BaseFrame::Ptr BaseFrame::create(cv::Mat img, const Pose &p, const CameraPinholeParams &cam)
{
	BaseFrame::Ptr bframe(new BaseFrame);
	bframe->image = img;
	bframe->mPose = p;
	bframe->cameraParam = cam;
	return bframe;
}


// Image dimensions
int
BaseFrame::width() const
{
	return cameraParam.width;
}


int
BaseFrame::height() const
{
	return cameraParam.height;
}


void
BaseFrame::projectPointCloud(
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointsInWorld,
		const CameraPinholeParams &camera,
		const Pose &cameraPose,
		const double cutDistance,
		MatrixProjectionResult &projRes)
{
	pcl::FrustumCulling<pcl::PointXYZ> frustum;

	frustum.setInputCloud(pointsInWorld);
	frustum.setNearPlaneDistance(0.5);
	frustum.setFarPlaneDistance(cutDistance);
	frustum.setHorizontalFOV(camera.getHorizontalFoV() * 180 / M_PI);
	frustum.setVerticalFOV(camera.getVerticalFoV() * 180 / M_PI);

	Eigen::Matrix4f cam2robot;
	cam2robot <<
			0, 0, 1, 0,
            0,-1, 0, 0,
            1, 0, 0, 0,
            0, 0, 0, 1;
	Matrix4f poseFilt = cameraPose.cast<float>() * cam2robot;
	frustum.setCameraPose(poseFilt);

	pcl::PointCloud<pcl::PointXYZ> filtered;
	frustum.filter(filtered);

	projRes.resize(filtered.size(), Eigen::NoChange);
	// Must use this matrix data type (dont use auto)
	Matrix<double,3,4> camMat4 = camera.toMatrix() * createExternalParamMatrix4(cameraPose);
	for (uint64 i=0; i<filtered.size(); ++i) {
		auto pp = filtered.at(i);
		Vector4d pv(pp.x, pp.y, pp.z, 1);
		Vector3d pj = camMat4 * pv;
		pj /= pj[2];
		projRes.row(i) = pj.hnormalized();
	}
}


/*
 * Project point cloud in World Coordinate (eg. PCL Map) to this frame
 */
void
BaseFrame::projectPointCloud(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointsInWorld,
	const double cutDistance,
	MatrixProjectionResult &projRes) const
{
	return projectPointCloud(pointsInWorld, cameraParam, mPose, cutDistance, projRes);
}


/*
 * Project/Render point cloud in World Coordinate using image of this frame
 */
cv::Mat
BaseFrame::projectPointCloud(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointsInWorld,
	const double cutDistance) const
{
	cv::Mat frameImage = this->image.clone();
	MatrixProjectionResult projRes;

//	projectPointCloud(pointsInWorld, cutDistance, projRes);
	projectPointCloud(pointsInWorld, cameraParam, mPose, cutDistance, projRes);
	for (int r=0; r<projRes.rows(); ++r) {
		cv::Point2f p2f (projRes(r,0), projRes(r,1));
		cv::circle(frameImage, p2f, 2, cv::Scalar(0,0,255));
	}

	return frameImage;
}


Eigen::Vector3d
BaseFrame::keypointn (kpid k) const
{
	Vector3d v = keypointh(k);
	return cameraParam.toMatrix3().inverse() * v;
}


void
BaseFrame::extractKeypointsAndFeatures (const cv::Mat &mask, std::vector<cv::KeyPoint> &keypointsInMask, cv::Mat &descriptorsInMask) const
{
	vector<uint32_t> kpIds;

	extractKeypointsAndFeatures(mask, kpIds);
	keypointsInMask.resize(kpIds.size());
	descriptorsInMask = cv::Mat(kpIds.size(), fDescriptors.cols, fDescriptors.type());

	for (int i=0; i<kpIds.size(); ++i) {
		auto id = kpIds[i];
		keypointsInMask[i] = fKeypoints.at(id);
		descriptorsInMask.row(i) = fDescriptors.row(id);
	}
}


/*
 * XXX: Unfinished
 */
void
BaseFrame::extractKeypointsAndFeatures (const cv::Mat &mask, std::vector<uint32_t> &keypointIds) const
{
	keypointIds.clear();
	for (int i=0; i<numOfKeyPoints(); ++i) {
		cv::Point2i kp = fKeypoints.at(i).pt;
		if (mask.at<int>(kp)==0)
			continue;
		keypointIds.push_back(static_cast<uint32_t>(i));
	}
}


g2o::Sim3
BaseFrame::toSim3() const
{
//	g2o::
}


g2o::SE3Quat
BaseFrame::toSE3Quat() const
{
	auto extMat = createExternalParamMatrix4(mPose);
	return g2o::SE3Quat(extMat.block<3,3>(0,0), extMat.block<3,1>(0,3));
}
