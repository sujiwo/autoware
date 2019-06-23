/*
 * LidarMapper.cpp
 *
 *  Created on: May 13, 2019
 *      Author: sujiwo
 */

#include <fstream>
#include <sstream>
#include <thread>
#include <algorithm>

#include "LidarMapper.h"


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

	LidarMapper::parseConfiguration(
		configPath.string(),
		globalMapperParameters,
		localMapperParameters,
		worldToMap,
		generalParams,
		rootConfiguration);

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

	// Making sure GNSS trajectory always available
	buildGnssTrajectory();
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
	doScan([&, this](int64 bagId)
		{ return scanResultCallback(bagId); }
	);

	// XXX: Temporary
	globalMapperProc->vehicleTrack.dump((workDir / "ndt.csv").string());
	auto vecTrack = graph->dumpTrajectory();
	vecTrack.dump((workDir/"track_optimized.csv").string());
}


/*
 * A variant of the above, only doing scans and logging.
 * Optimization will be performed later
 */
void
LidarMapper::buildScanOnly()
{
	auto resultLogFilename = workDir / "results.log";
	auto resultPcdMap = workDir / "result_map.pcd";

	// Check if result log file is valid
	if (boost::filesystem::exists(resultLogFilename) and
		last_write_time(resultLogFilename) > last_write_time(workDir/ConfigurationFilename)) {

		std::fstream logging;
		logging.open(resultLogFilename.string(), std::fstream::in);
		localMapperProc->readLog(logging);
		globalMapperProc->readLog(logging);
		cout << "Results from previous run loaded" << endl;
		optimizeOnly();

		auto pcdMap = graph->createPointCloud();
		pcl::io::savePCDFileBinary(resultPcdMap.string(), *pcdMap);
		cout << "Map dumped to " << resultPcdMap.string() << endl;

		dumpStatistics();
	}

	else {

		cout << "Result log file not valid; rescanning" << endl;
		doScan();

		std::fstream logging;
		logging.open(resultLogFilename.string(), std::fstream::out | std::fstream::trunc);
		localMapperProc->saveLog(logging);
		globalMapperProc->saveLog(logging);
	}
}


void
LidarMapper::optimizeOnly()
{
	assert(localMapperProc->scanResults.size() == globalMapperProc->scanResults.size());

	for (int64 bagId=generalParams.startId, c=0; bagId<=generalParams.stopId; ++bagId, ++c) {
		scanResultCallback(bagId);
		cout << c+1 << '/' << generalParams.stopId-generalParams.startId << "      \r" << flush;
	}

	globalMapperProc->vehicleTrack.dump((workDir / "ndt.csv").string());
	auto vecTrack = graph->dumpTrajectory();
	vecTrack.dump((workDir/"track_optimized.csv").string());
}


void
LidarMapper::scanResultCallback(int64 bagId)
{
	const auto &localScanLog = localMapperProc->getScanLog(bagId);
	const auto &globalScanLog = globalMapperProc->getScanLog(bagId);
	if (localScanLog.hasScanFrame==true) {
		addNewScanFrame(bagId);
	}

	// Check if we need to kick off Pose Graph Optimization
	if (elapsed_distance_for_optimization >= generalParams.optimization_distance_trigger or
		bagId==generalParams.stopId) {
		cout << "Optimization started" << endl;
		flushScanQueue();
		// XXX: Check
		graph->optimize(1024);
	}
}


void
LidarMapper::doScan(std::function<void(int64)> callback)
{
	for (int64 bagId=generalParams.startId, c=0; bagId<=generalParams.stopId; ++bagId, ++c) {

		ptime messageTime;
		auto currentScan4 = lidarBag->getUnfiltered<pcl::PointXYZI>(bagId, &messageTime);
		auto currentScan3 = lidarBag->getFiltered<pcl::PointXYZ>(bagId);

		thread local ([&, this] {
			localMapperProc->feed(currentScan4, messageTime, bagId);
		});

		thread global ([&, this] {
			globalMapperProc->feed(currentScan3, messageTime, bagId);
		});

		local.join();
		global.join();

		if (callback==nullptr) {}
		else callback(bagId);

		cout << c+1 << '/' << generalParams.stopId-generalParams.startId << "      \r" << flush;
	}
}


/*
 * Entry point to the whole mapping process
 */
int
LidarMapper::createMapFromBag(const std::string &bagpath, const std::string &workingDirectoryPath)
{
	LidarMapper lidarMapperInstance(bagpath, workingDirectoryPath);
	if (lidarMapperInstance.generalParams.scanOnly==false)
		lidarMapperInstance.build();
	else
		lidarMapperInstance.buildScanOnly();

	return 0;
}


void
LidarMapper::parseConfiguration(
	const std::string &configPath,
	GlobalMapper::Param &g,
	LocalMapper::Param &l,
	TTransform &worldToMap,
	LidarMapper::Param &gen,
	inipp::Ini<char> &ini)
{
	std::ifstream configFile(configPath);
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

	string startIdstr, stopIdstr;
	inipp::extract(ini.sections["General"]["start"], startIdstr);
	inipp::extract(ini.sections["General"]["stop"], stopIdstr);
	inipp::extract(ini.sections["General"]["scan only"], gen.scanOnly);

	gen.startInp = InputOffsetPosition::parseString(startIdstr);
	gen.stopInp = InputOffsetPosition::parseString(stopIdstr);

	inipp::extract(ini.sections["General"]["optimization_distance_trigger"], gen.optimization_distance_trigger);

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
LidarMapper::addNewScanFrame(int64 bagId)
{
	const auto &localScanLog = localMapperProc->getScanLog(bagId);
	const auto &globalScanLog = globalMapperProc->getScanLog(bagId);

	if (localScanLog.hasScanFrame==false)
		return;

	const ptime &t=localScanLog.timestamp;
	Pose gnssPose = getGnssPose(t);

	ScanFrame::Ptr newScan;
	double diffDistance = 0.0;

	if (bagId==generalParams.startId) {
		newScan = ScanFrame::create(bagId, t, gnssPose, gnssPose, localScanLog.accum_distance);
	}

	else {
		const auto &prevLocalLog = localMapperProc->getScanLog(localScanLog.prevScanFrame);
		TTransform odomMove = prevLocalLog.poseAtScan.inverse() * localScanLog.poseAtScan;
		Pose newpose = graph->lastFrame()->odometry * odomMove;
		newScan = ScanFrame::create(bagId, t, newpose, gnssPose, localScanLog.accum_distance);
		diffDistance = localScanLog.accum_distance - prevLocalLog.accum_distance;
	}

	elapsed_distance_for_optimization += diffDistance;
	graph->addScanFrame(newScan);
	scanFrameQueue.push_back(newScan);

	// XXX: thresholding here ?
	if (globalScanLog.fitness_score < 1.0) {
		cout << "Use global match for #" << bagId << endl;
		graph->addGlobalPose(newScan, globalScanLog.poseAtScan);
	}

	// use GNSS pose instead ?
	else {
		cout << "Use GNSS for #" << bagId << endl;
		graph->addGlobalPose(newScan, gnssPose);
	}
}


void
LidarMapper::flushScanQueue()
{
	if (scanFrameQueue.empty())
		return;

	cout << "Added " << scanFrameQueue.size() << " new frames" << endl;

	// Try to find loop at this point
	auto loopEvent = loopDetector->detect(graph->getFrameList(), scanFrameQueue);
	for (auto &loop: loopEvent) {
		graph->handleLoop(loop);
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


template<typename K, typename V>
std::vector<K> extractKeys(const std::map<K,V> &maps)
{
	vector<K> keys;
	keys.reserve(maps.size());
	for (auto &pair: maps) {
		keys.push_back(pair.first);
	}
	return keys;
}


void
LidarMapper::dumpStatistics()
{
	const auto statFilename = workDir / "scanning_stats.csv";

	std::fstream statfd(statFilename.string(), std::fstream::trunc|std::fstream::out);

	auto bagIds = extractKeys(localMapperProc->scanResults);
	for (auto bid: bagIds) {
		auto globalLog = globalMapperProc->getScanLog(bid);
		auto localLog = localMapperProc->getScanLog(bid);
		statfd 	<< bid << ' '
				<< globalLog.gnssIsUsed << ' '
				<< globalLog.fitness_score << ' '
				<< endl;
	}

	statfd.close();
}


}	// Namespace LidarMapper
