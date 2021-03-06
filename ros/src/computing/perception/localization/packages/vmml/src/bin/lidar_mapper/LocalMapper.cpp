/*
 * LocalMapper.cpp
 *
 *  Created on: May 15, 2019
 *      Author: sujiwo
 */

#include <boost/serialization/map.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "LidarMapper.h"


using pcl::PointCloud;
using pcl::PointXYZI;

using namespace std;
using namespace Eigen;


namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive &ar, LidarMapper::LocalMapper::ScanProcessLog &log, unsigned int version)
{
	ar
		& log.sequence_num
		& log.timestamp
		& log.numOfScanPoints
		& log.filteredScanPoints
		& log.mapNumOfPoints
		& log.hasConverged
		& log.fitness_score
		& log.transformation_probability
		& log.num_of_iteration
		& log.poseAtScan
		& log.shift
		& log.submap_id
		& log.submap_size
		& log.submap_origin_stamp
		& log.submap_origin_pose
		& log.hasScanFrame
		& log.prevScanFrame
		& log.accum_distance
		& log.matchingTime
		& log.currentVelocity;
}

}
}


namespace LidarMapper {


LocalMapper::LocalMapper(LidarMapper &_parent, const LocalMapper::Param &p):
	parent(_parent),
	param(p)
{
	auto iniCfg = parent.getRootConfig();

	// Parameter set
	inipp::extract(iniCfg.sections["Local Mapping"]["max_scan_range"], param.max_scan_range);
	mNdt.setResolution(param.ndt_res);
	mNdt.setStepSize(param.step_size);
	mNdt.setTransformationEpsilon(param.trans_eps);
	mNdt.setMaximumIterations(param.max_iter);

	mVoxelGridFilter.setLeafSize(param.voxel_leaf_size, param.voxel_leaf_size, param.voxel_leaf_size);

	auto submapsDir = parent.workDir / "submaps";
	if (!boost::filesystem::exists(submapsDir)) {
		boost::filesystem::create_directory(submapsDir);
	}
	else if (!boost::filesystem::is_directory(submapsDir))
		throw runtime_error("Not a directory: "+submapsDir.string());
}


void
LocalMapper::feed(LocalMapperCloud::ConstPtr newScan, const ptime &messageTime, int64 scanId)
{
	ScanProcessLog feedResult;

	current_scan_time = messageTime;
	currentScanId = scanId;

	feedResult.sequence_num = scanId;
	feedResult.timestamp = messageTime;

	try {
		if (scanId==parent.generalParams.startId) {
			feedResult.hasScanFrame = true;
			lastScanFrame = scanId;
		}

		// Add initial point cloud to velodyne_map
		if (initial_scan_loaded==false) {
			currentMap += *newScan;
			initial_scan_loaded = true;

			throw feedResult;
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

		ptime trun1 = getCurrentTime();
		mNdt.align(*output_cloud, guessPose.matrix().cast<float>());
		ptime trun2 = getCurrentTime();
		feedResult.matchingTime = trun2 - trun1;

		TTransform t_localizer = mNdt.getFinalTransformation().cast<double>();

		LocalMapperCloud::Ptr transformed_scan_ptr(new LocalMapperCloud);
		pcl::transformPointCloud(*newScan, *transformed_scan_ptr, t_localizer);

		Pose current_pose = t_localizer;

		if (hasSubmapIdIncremented==true) {
			submapOriginTimestamp = messageTime;
			submapOriginPose = current_pose;
			hasSubmapIdIncremented = false;
		}

		// Calculate the displacement (or offset) (current_pose - previous_pose)
		auto prevFrameLog = getScanLog(scanId-1);
		lastDisplacement = previous_pose.inverse() * current_pose;
		double linDisplacement, anglDisplacement;
		previous_pose.displacement(current_pose, linDisplacement, anglDisplacement);
		feedResult.currentVelocity = Twist(prevFrameLog.poseAtScan, current_pose, toSeconds(messageTime-prevFrameLog.timestamp));

		// Calculate shift (in X-Y plane only)
		double shift = Vector2d(current_pose.x()-added_pose.x(), current_pose.y()-added_pose.y()).norm();
		// Update the map when horizontal shift is larger than minimum range
		if (shift >= param.min_add_scan_shift) {

			auto lastMapShift = added_pose.inverse() * current_pose;

			// add to pose graph
			accum_distance += lastMapShift.translation().norm();
			feedResult.accum_distance = accum_distance;

			// tell parent to create new scanframe
			feedResult.hasScanFrame = true;
			feedResult.prevScanFrame = lastScanFrame;
			lastScanFrame = scanId;

			submap_size += shift;
			currentMap += *transformed_scan_ptr;
			currentSubmap += *transformed_scan_ptr;
			added_pose = current_pose;
			isMapUpdate = true;
		}

		// Update position
		previous_pose = current_pose;
		feedResult.poseAtScan = current_pose;
		feedResult.submap_id = submap_id;

		// Output submap file after a certain threshold
		if (submap_size >= param.max_submap_size) {
			if (currentSubmap.size() != 0) {

	//			outputCurrentSubmap();

				currentMap = currentSubmap;
				currentSubmap.clear();
				submap_size = 0;
			}

			submap_id++;
			hasSubmapIdIncremented = true;
		}

		localMapTrack.push_back(PoseStamped(current_pose, messageTime));
		feedResult.fitness_score = mNdt.getFitnessScore();
		feedResult.num_of_iteration = mNdt.getFinalNumIteration();
		feedResult.hasConverged = mNdt.hasConverged();
		feedResult.transformation_probability = mNdt.getTransformationProbability();

		throw feedResult;

//	scanResults[scanId] = feedResult;
	} catch (const ScanProcessLog &lg) {
		scanResults.insert(make_pair(scanId, lg));
	}
}


void LocalMapper::outputCurrentSubmap()
{
	auto submapPath = parent.workDir / "submaps";
	submapPath /= generateSubmapPcdName();
	cerr << "Outputting submap: " << submapPath.string() << endl;
	pcl::io::savePCDFileBinary(submapPath.string(), currentSubmap);
}


string LocalMapper::ScanProcessLog::dump()
{
	stringstream ss;


	return ss.str();
}


string LocalMapper::generateSubmapPcdName()
{
	stringstream ss;

	tduration td = submapOriginTimestamp - unixTime0;
	ss << td.total_seconds() << '_' << currentScanId << ".pcd";
	return ss.str();
}


void
LocalMapper::saveLog(std::fstream &output) const
{
	boost::archive::binary_oarchive logger(output);
	logger << scanResults;
}


void
LocalMapper::readLog(std::fstream &input)
{
	boost::archive::binary_iarchive logInput(input);
	logInput >> scanResults;
}



}	// namespace LidarMapper
