/*
 * LidarMapper.cpp
 *
 *  Created on: May 13, 2019
 *      Author: sujiwo
 */

#include <sstream>

#include "LidarMapper.h"
#include "inipp.h"


using namespace std;
using namespace Eigen;


namespace LidarMapper {


LidarMapper::LidarMapper(const GlobalMapper::Param &gp, const LocalMapper::Param &lp, const string &bagpath, const string &lidarCalibrationFilePath) :
	globalMapperParameters(gp),
	localMapperParameters(lp)
{
	bagFd.open(bagpath, rosbag::bagmode::Read);
	// XXX: Please confirm this `convention' of topic sentences
	gnssBag = RandomAccessBag::Ptr(new RandomAccessBag(bagFd, "/nmea_sentence"));
	lidarBag = LidarScanBag::Ptr(
		new LidarScanBag(bagFd,
			"/velodyne_packets",
			ros::TIME_MIN,
			ros::TIME_MAX,
			lidarCalibrationFilePath));
}


LidarMapper::~LidarMapper() {
	// TODO Auto-generated destructor stub
}


void
LidarMapper::build()
{
	// Build GNSS Trajectory
	createTrajectoryFromGnssBag(*gnssBag, gnssTrajectory, 7, worldToMap);

	const int bagsize = lidarBag->size();
	for (int i=0; i<bagsize; ++i) {

		ptime messageTime;
		auto currentScan = lidarBag->getUnfiltered(i, &messageTime);
	}
}


/*
 * Entry point to the whole mapping process
 */
int
LidarMapper::createMapFromBag(const string &bagpath, const std::string &configPath, const string &lidarCalibrationFilePath)
{
	GlobalMapper::Param globalParameters;
	LocalMapper::Param localParameters;
	TTransform worldToMapTransform;

	LidarMapper::parseConfiguration(configPath, globalParameters, localParameters, worldToMapTransform);

	LidarMapper lidarMapperInstance(globalParameters, localParameters, bagpath, lidarCalibrationFilePath);
	lidarMapperInstance.worldToMap = worldToMapTransform;

	lidarMapperInstance.build();

	return 0;
}


void
LidarMapper::parseConfiguration(const std::string &configPath, GlobalMapper::Param &g, LocalMapper::Param &l, TTransform &worldToMap)
{
	ifstream configFile(configPath);
	inipp::Ini<char> ini;
	ini.parse(configFile);

	inipp::extract(ini.sections["Local"]["ndt_res"], l.ndt_res);
	inipp::extract(ini.sections["Local"]["step_size"], l.step_size);
	inipp::extract(ini.sections["Local"]["trans_eps"], l.trans_eps);
	inipp::extract(ini.sections["Local"]["max_iter"], l.max_iter);
	inipp::extract(ini.sections["Local"]["voxel_leaf_size"], l.voxel_leaf_size);
	inipp::extract(ini.sections["Local"]["min_scan_range"], l.min_scan_range);
	inipp::extract(ini.sections["Local"]["min_add_scan_shift"], l.min_add_scan_shift);
	inipp::extract(ini.sections["Local"]["max_submap_size"], l.max_submap_size);

	inipp::extract(ini.sections["Global"]["ndt_res"], g.ndt_res);
	inipp::extract(ini.sections["Global"]["step_size"], g.step_size);
	inipp::extract(ini.sections["Global"]["trans_eps"], g.trans_eps);
	inipp::extract(ini.sections["Global"]["max_iter"], g.max_iter);
	inipp::extract(ini.sections["Global"]["queue_size"], g.queue_size);

	Vector3d vWorldToMapTf, oWorldToMapTf;
	inipp::extract(ini.sections["GNSS"]["x"], vWorldToMapTf.x());
	inipp::extract(ini.sections["GNSS"]["y"], vWorldToMapTf.y());
	inipp::extract(ini.sections["GNSS"]["z"], vWorldToMapTf.z());
	inipp::extract(ini.sections["GNSS"]["roll"], oWorldToMapTf.x());
	inipp::extract(ini.sections["GNSS"]["pitch"], oWorldToMapTf.y());
	inipp::extract(ini.sections["GNSS"]["yaw"], oWorldToMapTf.z());
	worldToMap = TTransform::from_XYZ_RPY(vWorldToMapTf, oWorldToMapTf.x(), oWorldToMapTf.y(), oWorldToMapTf.z());

	return;
}


}	// Namespace LidarMapper
