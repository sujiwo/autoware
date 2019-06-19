/*
 * LidarMapper.h
 *
 *  Created on: May 13, 2019
 *      Author: sujiwo
 */

#ifndef _LIDARMAPPER_H_
#define _LIDARMAPPER_H_

#include <string>
#include <deque>
#include <map>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/filesystem.hpp>

#include "PoseGraph.h"
#include "LoopDetector.h"

#include "inipp.h"
#include "utilities.h"
#include "Trajectory.h"
#include "RandomAccessBag.h"
#include "datasets/MeidaiBagDataset.h"
#include "datasets/LidarScanBag2.h"

#include "ScanFrame.h"

namespace LidarMapper {


const std::string
	ConfigurationFilename 		= "lidar_mapper.ini",
	LidarCalibrationFilename 	= "calibration.yaml",
	GlobalMapFilename			= "global_map.pcd";


class LidarMapper;


struct InputOffsetPosition {
	double asSecondsFromStart = -1;
	int64_t asPosition = -1;

	static InputOffsetPosition
	parseString(const std::string &s);

};



class LocalMapper {
public:

friend class LidarMapper;

	struct Param {
		double
			ndt_res,
			step_size,
			trans_eps;
		int
			max_iter;
		double
			voxel_leaf_size,
			min_scan_range,
			min_add_scan_shift,
			max_submap_size;
	};

	// XXX: Subject to change
	struct ScanProcessLog {
		uint64_t sequence_num;
		ptime timestamp;
		int numOfScanPoints, filteredScanPoints, mapNumOfPoints;
		bool hasConverged = false;
		float
			fitness_score 					= std::numeric_limits<float>::max(),
			transformation_probability 		= std::numeric_limits<float>::max();
		int num_of_iteration;
		Pose poseAtScan;
		float shift;
		double submap_size = 0;
		ptime submap_origin_stamp;
		Pose submap_origin_pose;

		bool hasScanFrame					= false;
		int64 prevScanFrame					= -1;

		std::string dump();
	};

	typedef pcl::PointCloud<pcl::PointXYZI> LocalMapperCloud;

	LocalMapper(LidarMapper &_parent, const Param &p);
	void feed(pcl::PointCloud<pcl::PointXYZI>::ConstPtr newScan, const ptime &messageTime, int64 scanId);
	void outputCurrentSubmap();


protected:
	Param param;
	LidarMapper &parent;

	// Need separate NDT instances due to possible different parameters
//	pcl_omp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> mNdt;
	pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> mNdt;
	// Need our own voxel grid filter
	pcl::VoxelGrid<pcl::PointXYZI> mVoxelGridFilter;

	// Counters
	bool initial_scan_loaded = false;
	uint32_t currentScanId = 0;

	LocalMapperCloud currentMap, currentSubmap;

	// States
	ptime current_scan_time;
	bool isMapUpdate = true;
	bool hasSubmapIdIncremented = true;
	Pose
		previous_pose = TTransform::Identity(),
		added_pose = TTransform::Identity();
	TTransform
		lastDisplacement = TTransform::Identity(),
		displacementFromOrigin = TTransform::Identity();
	double submap_size = 0;
	uint32_t submap_id = 0;
	ptime submapOriginTimestamp;
	Pose submapOriginPose;
	double accum_distance = 0.0;
	int64_t lastScanFrame = -1;

	Trajectory localMapTrack;

	std::string generateSubmapPcdName();

	std::map<int64, ScanProcessLog> scanResults;

};	// LidarMapper::LocalMapper




class GlobalMapper {
public:

	friend class LidarMapper;

	struct Param {
		double
			ndt_res,
			step_size,
			trans_eps;
		int
			max_iter,
			queue_size;
	};

	struct ScanProcessLog : public LocalMapper::ScanProcessLog
	{
		bool gnssIsUsed = false;
	};

	typedef pcl::PointCloud<pcl::PointXYZ> GlobalMapperCloud;

	GlobalMapper(LidarMapper &_parent, const Param &p);
	void loadMap(const std::string &point_cloud_map_filename);
	void feed(GlobalMapperCloud::ConstPtr &newscan, const ptime &messageTime, int scanId);

protected:
	Param param;
	LidarMapper &parent;
	uint32_t currentScanId = 0;

	GlobalMapperCloud::Ptr globalMap;
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> mNdt;
	Trajectory vehicleTrack;

	// States
	bool init_pose_set = false;
	Pose
		current_gnss_pose, previous_gnss_pose,
		previous_pose = TTransform::Identity(),
		added_pose = TTransform::Identity();
	TTransform
		lastDisplacement = TTransform::Identity();
	double
		fitness_score,
		transformation_probability;
	int
		num_iterations;

	std::map<int64, ScanProcessLog> scanResults;

};	// LidarMapper::GlobalMapper



class LidarMapper {
public:

	struct Param {
		// To be filled by configuration parser
		InputOffsetPosition startInp, stopInp;
		// To be filled after the bag is open
		int startId, stopId;

		// For loop detection
		double
			optimization_distance_trigger,
			min_edge_interval,
			accum_distance_thresh,
			max_loop_distance;
	};

	friend class GlobalMapper;
	friend class LocalMapper;
	friend class ScanOdometry;
	friend class LoopDetector;

	LidarMapper(const std::string &bagpath, const boost::filesystem::path &myWorkDir);
	virtual ~LidarMapper();

	void build();

	void buildGnssTrajectory();

//	static int createMapFromBag(const std::string &bagpath, const std::string &configPath, const std::string &lidarCalibrationFilePath);

	static int createMapFromBag(const std::string &bagpath, const std::string &workingDirectory);

	static void parseConfiguration(
		const std::string &configPath,
		GlobalMapper::Param &g,
		LocalMapper::Param &l,
		TTransform &worldToMap,
		LidarMapper::Param &generalParams,
		inipp::Ini<char> &ini);

	PoseStamped getGnssPose(const ptime &t) const;

	const Param& getParams() const
	{ return generalParams; }

	const inipp::Ini<char>& getRootConfig() const
	{ return rootConfiguration; }

protected:

	// Root configuration
	inipp::Ini<char> rootConfiguration;

	GlobalMapper::Param globalMapperParameters;
	LocalMapper::Param localMapperParameters;
	Param generalParams;

	std::shared_ptr<LocalMapper> localMapperProc;
	std::shared_ptr<GlobalMapper> globalMapperProc;

	TTransform worldToMap;

	Trajectory gnssTrajectory;

	// Bag access
	rosbag::Bag bagFd;
	RandomAccessBag::Ptr gnssBag;
	LidarScanBag2::Ptr lidarBag;

	boost::filesystem::path workDir;

	// List of scanframes
	std::deque<ScanFrame::Ptr> scanFrameQueue;
	std::deque<ScanFrame::Ptr> new_scanFrames;

	PoseGraph::Ptr graph;
	LoopDetector::Ptr loopDetector;

	// States
	double elapsed_distance_for_optimization = 0.0;

	void detectLoopInGnssTrajectory(std::vector<std::pair<uint32_t,uint32_t>> &) const;

	void addNewScanFrame(int64_t bId, ptime timestamp, Pose odom, double accum_distance, double fitnessScore=0.0);
	void flushScanQueue();
};

} // LidarMapper

#endif /* _LIDARMAPPER_H_ */
