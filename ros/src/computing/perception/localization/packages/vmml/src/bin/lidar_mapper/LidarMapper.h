/*
 * LidarMapper.h
 *
 *  Created on: May 13, 2019
 *      Author: sujiwo
 */

#ifndef _LIDARMAPPER_H_
#define _LIDARMAPPER_H_

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_omp_registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

#include "utilities.h"
#include "Trajectory.h"
#include "RandomAccessBag.h"
#include "datasets/MeidaiBagDataset.h"
#include "datasets/LidarScanBag2.h"


namespace LidarMapper {


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
	bool hasConverged;
	float fitness_score;
	int num_of_iteration;
	Pose poseAtScan;
	float shift;
	double submap_size = 0;
	ptime submap_origin_stamp;
	Pose submap_origin_pose;

	std::string dump();
};


LocalMapper(const Param &p);
void feed(pcl::PointCloud<pcl::PointXYZI>::ConstPtr newScan, const ptime &messageTime);


protected:
	Param param;

	// Need separate NDT instance due to possible different parameters
	pcl_omp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> mNdt;
	// Need our own voxel grid filter
	pcl::VoxelGrid<pcl::PointXYZI> mVoxelGridFilter;

	// Counters
	bool initial_scan_loaded = false;
	uint64_t currentScanId = 0;

	pcl::PointCloud<pcl::PointXYZI> currentMap, currentSubmap;

	// States
	bool isMapUpdate = true;
	bool hasSubmapIdIncremented = true;
	Pose
		previous_pose,
		added_pose = TTransform::Identity();
	TTransform
		lastDisplacement = TTransform::Identity(),
		displacementFromOrigin = TTransform::Identity();
	double submap_size = 0;
	uint64_t submap_id = 0;

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

GlobalMapper(const Param &p);
void loadMap(const std::string &point_cloud_map);
void feed(const pcl::PointCloud<pcl::PointXYZ> &newscan);

protected:
	Param param;
	uint64_t currentScanId = 0;

};	// LidarMapper::GlobalMapper



class LidarMapper {
public:

	friend class GlobalMapper;
	friend class LocalMapper;

	LidarMapper(const GlobalMapper::Param&, const LocalMapper::Param&, const std::string &bagpath, const std::string &lidarCalibrationFile);
	virtual ~LidarMapper();

	void build();

	static int createMapFromBag(const std::string &bagpath, const std::string &configPath, const std::string &lidarCalibrationFilePath);

	// Second variant
	static int createMapFromBag(const std::string &bagpath, const std::string &workingDirectory);

	static void parseConfiguration(const std::string &configPath, GlobalMapper::Param &g, LocalMapper::Param &l, TTransform &worldToMap);

protected:
	const GlobalMapper::Param globalMapperParameters;
	const LocalMapper::Param localMapperParameters;

	LocalMapper localMapperProc;
	GlobalMapper globalMapperProc;

	TTransform worldToMap;

	Trajectory gnssTrajectory;

	// Bag access
	rosbag::Bag bagFd;
	RandomAccessBag::Ptr gnssBag;
	LidarScanBag2::Ptr lidarBag;
};

} // LidarMapper

#endif /* _LIDARMAPPER_H_ */
