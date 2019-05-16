/*
 * LocalMapper.cpp
 *
 *  Created on: May 15, 2019
 *      Author: sujiwo
 */


#include <pcl/common/transforms.h>

#include "LidarMapper.h"


using pcl::PointCloud;
using pcl::PointXYZI;

using namespace std;


namespace LidarMapper {

typedef PointCloud<PointXYZI> LocalMapperCloud;


LocalMapper::LocalMapper(const LocalMapper::Param &p):
	param(p)
{
	// Parameter set
	mNdt.setResolution(param.ndt_res);
	mNdt.setStepSize(param.step_size);
	mNdt.setTransformationEpsilon(param.trans_eps);
	mNdt.setMaximumIterations(param.max_iter);

	mVoxelGridFilter.setLeafSize(param.voxel_leaf_size, param.voxel_leaf_size, param.voxel_leaf_size);

}


void
LocalMapper::feed(LocalMapperCloud::ConstPtr newScan, const ptime &messageTime)
{
	ScanProcessLog feedResult;

	LocalMapperCloud::Ptr scan_ptr(new LocalMapperCloud);
	pcl::copyPointCloud(*newScan, *scan_ptr);

	// Add initial point cloud to velodyne_map
	if (initial_scan_loaded==false) {
		currentMap += *newScan;
		initial_scan_loaded = true;
	}

	// Apply voxel grid filter
	LocalMapperCloud::Ptr filtered_scan_ptr (new LocalMapperCloud);
	mVoxelGridFilter.setInputCloud(newScan);
	mVoxelGridFilter.filter(*filtered_scan_ptr);

	LocalMapperCloud::Ptr map_ptr(new LocalMapperCloud(currentMap));

	mNdt.setInputSource(filtered_scan_ptr);

	if (isMapUpdate==true) {
		mNdt.setInputTarget(map_ptr);
		isMapUpdate = false;
	}

	// Guess Pose
	Vector3d rot = quaternionToRPY(lastDisplacement.orientation());
	TTransform guessDisplacement = TTransform::from_XYZ_RPY(lastDisplacement.translation(), 0, 0, rot.z());
	Pose guessPose = previous_pose * guessDisplacement;

	LocalMapperCloud::Ptr output_cloud(new LocalMapperCloud);
	mNdt.align(*output_cloud, guessPose.matrix().cast<float>());

	TTransform t_localizer = mNdt.getFinalTransformation().cast<double>();

	// XXX: What if we use the same point cloud ?
	pcl::transformPointCloud(*scan_ptr, *scan_ptr, t_localizer);

	feedResult.fitness_score = mNdt.getFitnessScore();

	Pose current_pose = t_localizer;

	if (hasSubmapIdIncremented==true) {
		feedResult.submap_origin_stamp = messageTime;
		feedResult.submap_origin_pose = current_pose;
		hasSubmapIdIncremented = false;
	}

	// Calculate the displacement (current_pose - previous_pose)
	lastDisplacement = previous_pose.inverse() * current_pose;

	// Update position
	previous_pose = current_pose;

	// Calculate shift
	// XXX: Unfinished

	// End
	currentScanId += 1;
}


string LocalMapper::ScanProcessLog::dump()
{
	stringstream ss;


	return ss.str();
}


}	// namespace LidarMapper
