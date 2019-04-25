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


NdtLocalizer2::NdtLocalizer2()
{
	mNdt.setMaximumIterations(mParams.maximum_iterations);
	mNdt.setTransformationEpsilon(mParams.transformation_epsilon);
	mNdt.setStepSize(mParams.step_size);
	mNdt.setResolution(mParams.ndt_resolution);
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


void
NdtLocalizer2::localizeFromBag (LidarScanBag &bagsrc, Trajectory &resultTrack, const Trajectory &gnssTrack, const std::string &pcdMapFile)
{
	bagsrc.filtered = true;
	NdtLocalizer2 lidarLocalizer;
	lidarLocalizer.loadMap(pcdMapFile);
	resultTrack.clear();

	// Initialize
	ptime t0;
	uint32_t i0, i1;
	auto scan0 = bagsrc.at(0, &t0);
	PoseStamped lidarp0 = gnssTrack.interpolate(t0, &i0, &i1);
	Twist velocity (gnssTrack.at(i0), gnssTrack.at(i1));

	for (uint32_t i=1; i<bagsrc.size(); ++i) {

	}

	return;
}
