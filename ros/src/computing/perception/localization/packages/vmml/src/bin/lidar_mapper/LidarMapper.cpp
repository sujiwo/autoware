/*
 * LidarMapper.cpp
 *
 *  Created on: May 13, 2019
 *      Author: sujiwo
 */

#include <fstream>
#include <sstream>
#include <thread>

#include "LidarMapper.h"
#include "inipp.h"


using namespace std;
using namespace Eigen;
using namespace boost::filesystem;


namespace LidarMapper {


LidarMapper::LidarMapper(const std::string &bagpath, const boost::filesystem::path &myWorkDir)
{
	workDir = myWorkDir;
	if (is_directory(workDir)==false)
		throw runtime_error("Unable to open work directory");

	path
		configPath = workDir / ConfigurationFilename,
		lidarCalibrationPath = workDir / LidarCalibrationFilename;

	if (is_regular_file(configPath)==false)
		throw runtime_error("Configuration file does not exist at work directory");
	if (is_regular_file(lidarCalibrationPath)==false)
		throw runtime_error("Calibration file does not exist at work directory");

	LidarMapper::parseConfiguration(configPath.string(), globalMapperParameters, localMapperParameters, worldToMap, generalParams);

	localMapperProc = shared_ptr<LocalMapper>(new LocalMapper(*this, localMapperParameters));
	globalMapperProc = shared_ptr<GlobalMapper>(new GlobalMapper(*this, globalMapperParameters));

	bagFd.open(bagpath, rosbag::bagmode::Read);
	gnssBag = RandomAccessBag::Ptr(new RandomAccessBag(bagFd, "/nmea_sentence"));
	lidarBag = LidarScanBag2::Ptr(
		new LidarScanBag2(bagFd,
			"/velodyne_packets",
			ros::TIME_MIN,
			ros::TIME_MAX,
			lidarCalibrationPath.string(),
			localMapperParameters.min_scan_range
			));
	cout << "Bag ready" << endl;

	if (generalParams.startInp.asSecondsFromStart!=-1) {
		generalParams.startId = lidarBag->getPositionAtDurationSecond(generalParams.startInp.asSecondsFromStart);
	}
	else if (generalParams.startInp.asPosition!=-1) {
		generalParams.startId = generalParams.startInp.asPosition;
	}
	else {
		generalParams.startId = 0;
	}

	if (generalParams.stopInp.asSecondsFromStart!=-1) {
		generalParams.stopId = lidarBag->getPositionAtDurationSecond(generalParams.stopInp.asSecondsFromStart);
	}
	else if (generalParams.stopInp.asPosition!=-1) {
		generalParams.stopId = generalParams.stopInp.asPosition;
	}
	else if (generalParams.stopInp.asPosition==-1 and generalParams.stopInp.asSecondsFromStart==-1) {
		generalParams.stopId = lidarBag->size() - 1;
	}

	cout << "Sequence ID: " << generalParams.startId << "->" << generalParams.stopId << endl;

	// Other important objects
	graph = PoseGraph::Ptr(new PoseGraph(*this));
	loopDetector = LoopDetector::Ptr(new LoopDetector(*this));
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
	buildGnssTrajectory();

	for (int i=generalParams.startId, c=0; i<=generalParams.stopId; ++i, ++c) {

		ptime messageTime;
		auto currentScan4 = lidarBag->getUnfiltered<pcl::PointXYZI>(i, &messageTime);
		auto currentScan3 = lidarBag->getFiltered<pcl::PointXYZ>(i);

		thread local ([&, this] {
			localMapperProc->feed(currentScan4, messageTime, i);
		});

		thread global ([&, this] {
			globalMapperProc->feed(currentScan3, messageTime, i);
		});

		local.join();
		global.join();

		// Check if we need to kick off Pose Graph Optimization
		if (elapsed_distance_for_optimization >= generalParams.optimization_distance_trigger) {
			cout << "Optimization started" << endl;
			flushScanQueue();
		}

		cout << c+1 << '/' << generalParams.stopId-generalParams.startId << "      \r" << flush;
	}

	// XXX: Temporary
	globalMapperProc->vehicleTrack.dump((workDir / "ndt.csv").string());
}


/*
 * Entry point to the whole mapping process
 */
int
LidarMapper::createMapFromBag(const std::string &bagpath, const std::string &workingDirectoryPath)
{
	LidarMapper lidarMapperInstance(bagpath, workingDirectoryPath);
	lidarMapperInstance.build();

	return 0;
}


void
LidarMapper::parseConfiguration(
	const std::string &configPath,
	GlobalMapper::Param &g,
	LocalMapper::Param &l,
	TTransform &worldToMap,
	LidarMapper::Param &gen)
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
	inipp::extract(ini.sections["GNSS"]["stddev_horizontal"], gen.gnss_stddev_horizontal);
	inipp::extract(ini.sections["GNSS"]["stddev_vertical"], gen.gnss_stddev_vertical);

	string startIdstr, stopIdstr;
	inipp::extract(ini.sections["General"]["start"], startIdstr);
	inipp::extract(ini.sections["General"]["stop"], stopIdstr);

	gen.startInp = InputOffsetPosition::parseString(startIdstr);
	gen.stopInp = InputOffsetPosition::parseString(stopIdstr);

	inipp::extract(ini.sections["General"]["optimization_distance_trigger"], gen.optimization_distance_trigger);
	inipp::extract(ini.sections["General"]["min_edge_interval"], gen.min_edge_interval);
	inipp::extract(ini.sections["General"]["accum_distance_thresh"], gen.accum_distance_thresh);
	inipp::extract(ini.sections["General"]["max_loop_distance"], gen.max_loop_distance);

	return;
}


InputOffsetPosition
InputOffsetPosition::parseString(const std::string &s)
{
	InputOffsetPosition inp;
	if (s.find('.') != std::string::npos) {
		inp.asSecondsFromStart = stod(s);
	}
	else {
		inp.asPosition = stoi(s);
	}
	return inp;
}


PoseStamped
LidarMapper::getGnssPose(const ptime &t) const
{
	if (t < gnssTrajectory.front().timestamp or t > gnssTrajectory.back().timestamp) {
		return gnssTrajectory.extrapolate(t);
	}
	else {
		return gnssTrajectory.interpolate(t);
	}
}


void
LidarMapper::addNewScanFrame(int64_t bId, ptime timestamp, Pose odom, double accum_distance)
{
	Pose gnssPose = getGnssPose(timestamp);
	auto scanfr = ScanFrame::create(bId, timestamp, odom, gnssPose, accum_distance);
	// G2O node will be computed later
	scanFrameQueue.push_back(scanfr);
	elapsed_distance_for_optimization += accum_distance;
}


void
LidarMapper::flushScanQueue()
{
	if (scanFrameQueue.empty())
		return;

	for (auto &frame: scanFrameQueue) {
		graph->addScanFrame(frame);
	}

	// Try to find loop at this point
	auto loopEvent = loopDetector->detect(graph->getFrameList(), scanFrameQueue);
	for (auto &loop: loopEvent) {
		// XXX: Unfinished
	}
	elapsed_distance_for_optimization = 0.0;

	scanFrameQueue.clear();
}


void
LidarMapper::buildGnssTrajectory()
{
	// Build GNSS Trajectory for the whole track
	createTrajectoryFromGnssBag(*gnssBag, gnssTrajectory, 7, worldToMap);

	ptime
		startTime = lidarBag->timeAt(generalParams.startId).toBoost(),
		stopTime = lidarBag->timeAt(generalParams.stopId-1).toBoost();

	// For debugging
	Trajectory gnssSubsetTrack;
	for (int i=generalParams.startId; i<=generalParams.stopId; ++i) {
		auto cPose = getGnssPose(lidarBag->timeAt(i).toBoost());
		gnssSubsetTrack.push_back(cPose);
	}

	gnssTrajectory.dump((workDir / "gnss-full.csv").string());
	gnssSubsetTrack.dump((workDir / "gnss.csv").string());
}


}	// Namespace LidarMapper
