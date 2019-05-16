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

struct ScanProcessLog {
	dataItemId sequence_num;
	ptime timestamp;
	int numOfScanPoints, filteredScanPoints, mapNumOfPoints;
	bool hasConverged;
	float fitness_score;
	int num_of_iteration;
	Pose poseAtScan;
	float shift;
	double submap_size;

	std::string dump();
};


LocalMapper(const Param &p);
void feed(LidarScanBag::scan_t::ConstPtr &newscan);


protected:
	Param param;

	// Need separate NDT instance due to possible different parameters
	pcl_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> mNdt;
	// Need our own voxel grid filter
	pcl::VoxelGrid<pcl::PointXYZI> mVoxelGridFilter;

	// Counters
	dataItemId currentScanId = 0;

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
void feed();

protected:
	dataItemId currentScanId = 0;

};	// LidarMapper::GlobalMapper



class LidarMapper {
public:

	friend class GlobalMapper;
	friend class LocalMapper;

	LidarMapper(const GlobalMapper::Param&, const LocalMapper::Param&, const std::string &bagpath, const std::string &lidarCalibrationFile);
	virtual ~LidarMapper();

	void build();

	static int createMapFromBag(const std::string &bagpath, const std::string &configPath, const std::string &lidarCalibrationFilePath);

	static void parseConfiguration(const std::string &configPath, GlobalMapper::Param &g, LocalMapper::Param &l, TTransform &worldToMap);

protected:
	const GlobalMapper::Param globalMapperParameters;
	const LocalMapper::Param localMapperParameters;

	TTransform worldToMap;

	Trajectory gnssTrajectory;

	// Bag access
	rosbag::Bag bagFd;
	RandomAccessBag::Ptr gnssBag;
	LidarScanBag::Ptr lidarBag;
};

} // LidarMapper

#endif /* _LIDARMAPPER_H_ */
