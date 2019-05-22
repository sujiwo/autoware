/*
 * GlobalMapper.cpp
 *
 *  Created on: May 16, 2019
 *      Author: sujiwo
 */

#include <pcl/io/pcd_io.h>

#include "LidarMapper.h"


using namespace std;


namespace LidarMapper {


GlobalMapper::GlobalMapper(LidarMapper &_parent, const GlobalMapper::Param &p) :
	parent(_parent),
	param(p),
	globalMap(new GlobalMapperCloud)
{
	auto pcdmap = parent.workDir / GlobalMapFilename;
	loadMap(pcdmap.string());
}


void GlobalMapper::loadMap(const string &pcdFilename)
{
	pcl::PCDReader fReader;
	if (fReader.read(pcdFilename, *globalMap) != 0)
		throw runtime_error("Unable to open map file: "+pcdFilename);
	mNdt.setInputTarget(globalMap);
}


void GlobalMapper::feed(GlobalMapperCloud::ConstPtr &newScan, const ptime &messageTime)
{
	// Do nothing when prior map is not available
	if (globalMap->empty()==true)
		return;

	// see if position is available in GNSS trajectory
	Pose gnssPose;
	if (messageTime < parent.gnssTrajectory.front().timestamp) {
		if (toSeconds(parent.gnssTrajectory.front().timestamp-messageTime) > 0.1)
			return;
		gnssPose = parent.gnssTrajectory.extrapolate(messageTime);
	}
	else if (messageTime > parent.gnssTrajectory.back().timestamp) {

	}
	else {
		gnssPose = parent.gnssTrajectory.interpolate(messageTime);
	}

	// XXX: Unfinished
}

}	// namespace LidarMapper


