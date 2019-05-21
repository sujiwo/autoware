/*
 * LocalMapper.cpp
 *
 *  Created on: May 15, 2019
 *      Author: sujiwo
 */


#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "LidarMapper.h"


using pcl::PointCloud;
using pcl::PointXYZI;

using namespace std;
using namespace Eigen;


namespace LidarMapper {


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
	// XXX: Debug
	bool __debug=false;
	if (__debug==true)
		pcl::io::savePCDFileBinary("/tmp/debug.pcd", *newScan);

	ScanProcessLog feedResult;

	current_scan_time = messageTime;

	// Add initial point cloud to velodyne_map
	if (initial_scan_loaded==false) {
		currentMap += *newScan;
		initial_scan_loaded = true;
		return;
	}

	// Apply voxel grid filter.
	// The result of this filter will be used to match current submap
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
	feedResult.fitness_score = mNdt.getFitnessScore();

	TTransform t_localizer = mNdt.getFinalTransformation().cast<double>();

	LocalMapperCloud::Ptr transformed_scan_ptr(new LocalMapperCloud);
	pcl::transformPointCloud(*newScan, *transformed_scan_ptr, t_localizer);

	Pose current_pose = t_localizer;

	if (hasSubmapIdIncremented==true) {
		feedResult.submap_origin_stamp = messageTime;
		feedResult.submap_origin_pose = current_pose;
		hasSubmapIdIncremented = false;
	}

	// Calculate the displacement (or offset) (current_pose - previous_pose)
	lastDisplacement = previous_pose.inverse() * current_pose;

	// Update position
	previous_pose = current_pose;

	// Calculate shift (in X-Y plane only)
	double shift = Vector2d(current_pose.x()-added_pose.x(), current_pose.y()-added_pose.y()).norm();
	if (shift >= param.min_add_scan_shift) {
		submap_size += shift;
		currentMap += *transformed_scan_ptr;
		currentSubmap += *transformed_scan_ptr;
		added_pose = current_pose;
		isMapUpdate = true;
	}

	// XXX: Do some output to CSV here

	// Output submap file after a certain threshold
	if (submap_size >= param.max_submap_size) {
		if (currentSubmap.size() != 0) {
			// XXX: Output the PCD
			cerr << "Outputting submap" << endl;
			pcl::io::savePCDFileBinary("/tmp/test_submap.pcd", currentSubmap);

			currentMap = currentSubmap;
			currentSubmap.clear();
			submap_size = 0;
		}

		submap_id++;
		hasSubmapIdIncremented = true;
	}
	// XXX: Put logging here

	// End
	currentScanId += 1;
	cerr << currentScanId << ' ' << shift << endl;
}


string LocalMapper::ScanProcessLog::dump()
{
	stringstream ss;


	return ss.str();
}


}	// namespace LidarMapper
