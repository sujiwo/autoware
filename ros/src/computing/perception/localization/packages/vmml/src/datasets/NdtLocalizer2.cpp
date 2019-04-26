/*
 * NdtLocalizer2.cpp
 *
 *  Created on: Apr 24, 2019
 *      Author: sujiwo
 */



#include <string>
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
NdtLocalizer2::getTransform(const LidarScanBag::scan_t &scan1, const LidarScanBag::scan_t &scan2)
{
	pcl_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> mNdt;

}


void
NdtLocalizer2::localizeFromBag (LidarScanBag &bagsrc, Trajectory &resultTrack, const Trajectory &gnssTrack, const std::string &pcdMapFile)
{
	bagsrc.filtered = true;
	resultTrack.clear();

	// Initialize
	ptime t0;
	uint32_t i0, i1;
	auto scan0 = bagsrc.at(0, &t0);
	PoseStamped lidarp0 = gnssTrack.interpolate(t0, &i0, &i1);
	Twist velocity (gnssTrack.at(i0), gnssTrack.at(i1));

	NdtLocalizer2 lidarLocalizer(lidarp0);
	lidarLocalizer.loadMap(pcdMapFile);

	for (uint32_t i=1; i<bagsrc.size(); ++i) {

	}

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
	TTransform ctrlWithNoise = ctrl;

	// Add noise to control
	double
		xshift = PF::nrand(ControlShiftStdDev),
		yshift = PF::nrand(ControlShiftStdDev),
		zshift = PF::nrand(0.5*ControlShiftStdDev);
	ctrlWithNoise.shift(xshift, yshift, zshift);

	return vstate * ctrlWithNoise;
}


double
NdtLocalizer2::measurementModel(const Pose &state, const vector<Pose> &observations) const
{
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

