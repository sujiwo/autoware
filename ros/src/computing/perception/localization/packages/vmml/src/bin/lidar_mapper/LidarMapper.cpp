/*
 * LidarMapper.cpp
 *
 *  Created on: May 13, 2019
 *      Author: sujiwo
 */

#include <fstream>
#include <sstream>

#include "LidarMapper.h"
#include "inipp.h"


using namespace std;
using namespace Eigen;
using namespace boost::filesystem;


namespace LidarMapper {


LidarMapper::LidarMapper(const GlobalMapper::Param &gp, const LocalMapper::Param &lp, const string &bagpath, const string &lidarCalibrationFilePath) :
	globalMapperParameters(gp),
	localMapperParameters(lp),

	localMapperProc(*this, localMapperParameters),
	globalMapperProc(*this, globalMapperParameters)
{
	bagFd.open(bagpath, rosbag::bagmode::Read);

	// XXX: Please confirm this `convention' of topic sentences
	gnssBag = RandomAccessBag::Ptr(new RandomAccessBag(bagFd, "/nmea_sentence"));
	lidarBag = LidarScanBag2::Ptr(
		new LidarScanBag2(bagFd,
			"/velodyne_packets",
			ros::TIME_MIN,
			ros::TIME_MAX,
			lidarCalibrationFilePath,
			localMapperParameters.min_scan_range));
}


LidarMapper::~LidarMapper() {
	// TODO Auto-generated destructor stub
}


/*
 * Core routine
 */
void
LidarMapper::build()
{
	// Build GNSS Trajectory
	createTrajectoryFromGnssBag(*gnssBag, gnssTrajectory, 7, worldToMap);

	const int bagsize = lidarBag->size();
	for (int i=0; i<bagsize; ++i) {

		ptime messageTime;
		auto currentScan4 = lidarBag->getUnfiltered<pcl::PointXYZI>(i, &messageTime);
		auto currentScan3 = lidarBag->getFiltered<pcl::PointXYZ>(i);

		// XXX: Currently focused on local mapping
		localMapperProc.feed(currentScan4, messageTime);
		globalMapperProc.feed(currentScan3, messageTime);
	}
}


/*
 * Entry point to mapping process
 * XXX: to be deprecated
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


/*
 * Entry point to the whole mapping process
 */
int
LidarMapper::createMapFromBag(const std::string &bagpath, const std::string &workingDirectory)
{
	GlobalMapper::Param globalParameters;
	LocalMapper::Param localParameters;
	TTransform worldToMapTransform;

	path workDir(workingDirectory);
	if (is_directory(workDir)==false)
		throw runtime_error("Unable to open work directory");

	path
		configPath = workDir / ConfigurationFilename,
		lidarCalibrationPath = workDir / LidarCalibrationFilename;

	if (is_regular_file(configPath)==false)
		throw runtime_error("Configuration file does not exist at work directory");
	if (is_regular_file(lidarCalibrationPath)==false)
		throw runtime_error("Calibration file does not exist at work directory");

	LidarMapper::parseConfiguration(configPath.string(), globalParameters, localParameters, worldToMapTransform);

	LidarMapper lidarMapperInstance(globalParameters, localParameters, bagpath, lidarCalibrationPath.string());
	lidarMapperInstance.worldToMap = worldToMapTransform;
	lidarMapperInstance.workDir = workDir;

	lidarMapperInstance.build();

	return 0;
}


void
LidarMapper::parseConfiguration(const std::string &configPath, GlobalMapper::Param &g, LocalMapper::Param &l, TTransform &worldToMap)
{
	std::ifstream configFile(configPath);
	inipp::Ini<char> ini;
	ini.parse(configFile);

	inipp::extract(ini.sections["Local Mapping"]["ndt_res"], l.ndt_res);
	inipp::extract(ini.sections["Local Mapping"]["step_size"], l.step_size);
	inipp::extract(ini.sections["Local Mapping"]["trans_eps"], l.trans_eps);
	inipp::extract(ini.sections["Local Mapping"]["max_iter"], l.max_iter);
	inipp::extract(ini.sections["Local Mapping"]["voxel_leaf_size"], l.voxel_leaf_size);
	inipp::extract(ini.sections["Local Mapping"]["min_scan_range"], l.min_scan_range);
	inipp::extract(ini.sections["Local Mapping"]["min_add_scan_shift"], l.min_add_scan_shift);
	inipp::extract(ini.sections["Local Mapping"]["max_submap_size"], l.max_submap_size);

	inipp::extract(ini.sections["Global Matching"]["ndt_res"], g.ndt_res);
	inipp::extract(ini.sections["Global Matching"]["step_size"], g.step_size);
	inipp::extract(ini.sections["Global Matching"]["trans_eps"], g.trans_eps);
	inipp::extract(ini.sections["Global Matching"]["max_iter"], g.max_iter);
	inipp::extract(ini.sections["Global Matching"]["queue_size"], g.queue_size);

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
