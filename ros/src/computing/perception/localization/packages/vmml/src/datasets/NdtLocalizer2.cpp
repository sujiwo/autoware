/*
 * NdtLocalizer2.cpp
 *
 *  Created on: Apr 24, 2019
 *      Author: sujiwo
 */



#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>

#include "NdtLocalizer2.h"

using namespace std;
using namespace Eigen;

const int ParticleNumber = 500;

const double
	GnssPlaneStdDev = 		2.0,
	// Need to be larger
	GnssVertStdDev = 		0.5,
	ControlShiftStdDev =	0.5;


NdtLocalizer2::NdtLocalizer2(const Pose &initialEstimation) :

	initPose(initialEstimation),
	pFilter(ParticleNumber, *this)

{
	mNdt.setMaximumIterations(mParams.maximum_iterations);
	mNdt.setTransformationEpsilon(mParams.transformation_epsilon);
	mNdt.setStepSize(mParams.step_size);
	mNdt.setResolution(mParams.ndt_resolution);

	pFilter.initializeParticles();
}


NdtLocalizer2::~NdtLocalizer2() {
}


void
NdtLocalizer2::loadMap (const std::string &filename)
{
	pcMap = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader fReader;
	fReader.read(filename, *pcMap);
	mNdt.setInputTarget(pcMap);

	cout << "Map loaded\n";
}


TTransform
NdtLocalizer2::getTransform(
	const LidarScanBag::scan_t::ConstPtr &scan1,
	const LidarScanBag::scan_t::ConstPtr &scan2,
	bool &isConverged)
{
	pcl_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> mNdt;

	mNdt.setTransformationEpsilon(0.01);
	mNdt.setStepSize(0.1);
	mNdt.setResolution(1.0);
	mNdt.setMaximumIterations(30);
	mNdt.setInputSource(scan1);
	mNdt.setInputTarget(scan2);

	LidarScanBag::scan_t finalCl;
	mNdt.align(finalCl);

	isConverged = mNdt.hasConverged();
	if (isConverged) return mNdt.getFinalTransformation();
	else return TTransform::Identity();
}


void
NdtLocalizer2::localizeFromBag (LidarScanBag &bagsrc, Trajectory &resultTrack, const Trajectory &gnssTrack, const std::string &pcdMapFile)
{
	bagsrc.filtered = true;
	resultTrack.clear();

	// Initialize
	ptime t0;
	uint32_t i0, i1;
	auto scan1 = bagsrc.at(0, &t0);
	PoseStamped lidarp0;
	// Unused at this time
	Twist velocity;

	if (t0 < gnssTrack.front().timestamp) {
		lidarp0 = gnssTrack.extrapolate(t0);
	}

	else {
		lidarp0 = gnssTrack.interpolate(t0, &i0, &i1);
		velocity = Twist(gnssTrack.at(i0), gnssTrack.at(i1));
	}

	Pose isx = lidarp0;

	NdtLocalizer2 lidarLocalizer(lidarp0);
	lidarLocalizer.loadMap(pcdMapFile);
	lidarLocalizer.lastLocalizationTime = t0;

//	auto scan1 = bagsrc.at(0, &lidarLocalizer.lastLocalizationTime);

	for (uint32_t i=1; i<bagsrc.size(); ++i) {
		ptime currentTime;
		auto scan2 = bagsrc.at(i, &currentTime);

		bool isConverged;
		TTransform t1to2 = NdtLocalizer2::getTransform(scan1, scan2, isConverged);

		// XXX: do not use at(), but interpolate or extrapolate
		Pose gpsPose;
		if (currentTime>=gnssTrack.front().timestamp and currentTime<=gnssTrack.back().timestamp)
			gpsPose = gnssTrack.at(currentTime);
		else
			gpsPose = gnssTrack.extrapolate(currentTime);
		vector<Pose> gpsFixes {gpsPose};
		lidarLocalizer.pFilter.update(t1to2, gpsFixes);

		// After localizing
		// Make average, then store
		isx = isx * t1to2;
		PoseStamped curPoseAvg = lidarLocalizer.summarizePf(currentTime, isx.orientation());
		resultTrack.push_back(curPoseAvg);

		scan1 = scan2;
		lidarLocalizer.lastLocalizationTime = currentTime;

		cout << i << " / " << bagsrc.size() << '\r' << flush;
	}
	cout << endl;

	return;
}


Pose
NdtLocalizer2::initializeParticleState() const
{
	Pose cstate = initPose;

	// Randomize
	cstate.x() += PF::nrand(GnssPlaneStdDev);
	cstate.y() += PF::nrand(GnssPlaneStdDev);
	cstate.z() += PF::nrand(GnssVertStdDev);

	return cstate;
}


Pose
NdtLocalizer2::motionModel(const Pose &vstate, const TTransform &ctrl) const
{
	Pose curPose = vstate * ctrl;

	// Add noise to current pose
	double
		xshift = PF::nrand(ControlShiftStdDev),
		yshift = PF::nrand(ControlShiftStdDev),
		zshift = PF::nrand(0.5*ControlShiftStdDev);

	Pose poseWithNoise = curPose.shift(xshift, yshift, zshift);

	return poseWithNoise;
}


double
NdtLocalizer2::measurementModel(const Pose &state, const vector<Pose> &observations) const
{
	// XXX: Only considers single observation
	const Pose &gnssPose = observations[0];

	double observationWeight = 0;
	double wo,
		xdiff = gnssPose.x() - state.x(),
		ydiff = gnssPose.y() - state.y(),
		zdiff = gnssPose.z() - state.z();

	return exp(-(
		(xdiff*xdiff / (2*GnssPlaneStdDev*GnssPlaneStdDev)) +
		(ydiff*ydiff / (2*GnssPlaneStdDev*GnssPlaneStdDev)) +
		(zdiff*zdiff / (2*GnssVertStdDev*GnssVertStdDev))
	));
}


PoseStamped
NdtLocalizer2::summarizePf(const ptime &t, const Quaterniond &orientationSet)
{
	vector<Pose*> posePtrList(pFilter.getNumberOfParticles());
	pFilter.getStates(posePtrList);

	Vector3d avgstatePos(0, 0, 0);
	for (auto p: posePtrList) {
		avgstatePos.x() += p->x();
		avgstatePos.y() += p->y();
		avgstatePos.z() += p->z();
	}

	avgstatePos.x() /= double(posePtrList.size());
	avgstatePos.y() /= double(posePtrList.size());
	avgstatePos.z() /= double(posePtrList.size());

	return PoseStamped(avgstatePos, orientationSet, t);
}
