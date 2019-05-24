/*
 * GlobalMapper.cpp
 *
 *  Created on: May 16, 2019
 *      Author: sujiwo
 */

#include <pcl/io/pcd_io.h>

#include "LidarMapper.h"


using namespace std;


namespace LidarMapper {

const double PREDICT_POSE_THRESHOLD = 0.5;


GlobalMapper::GlobalMapper(LidarMapper &_parent, const GlobalMapper::Param &p) :
	parent(_parent),
	param(p),
	globalMap(new GlobalMapperCloud)
{
	auto pcdmap = parent.workDir / GlobalMapFilename;
	loadMap(pcdmap.string());

	mNdt.setResolution(param.ndt_res);
	mNdt.setStepSize(param.step_size);
	mNdt.setTransformationEpsilon(param.trans_eps);
	mNdt.setMaximumIterations(param.max_iter);
}


void GlobalMapper::loadMap(const string &pcdFilename)
{
	pcl::PCDReader fReader;
	if (fReader.read(pcdFilename, *globalMap) != 0)
		return;
//		throw runtime_error("Unable to open map file: "+pcdFilename);
	pcl::transformPointCloud(*globalMap, *globalMap, parent.worldToMap.matrix().cast<float>());
	cout << "Initial Map loaded" << endl;

	mNdt.setInputTarget(globalMap);
}


void GlobalMapper::feed(GlobalMapperCloud::ConstPtr &newScan, const ptime &messageTime)
{
	// Do nothing when prior map is not available
	if (globalMap->empty()==true)
		return;

	// see if position is available in GNSS trajectory
	Pose guessPose, currentPose;
	if (currentScanId<=1) {
		if (messageTime < parent.gnssTrajectory.front().timestamp) {
			if (toSeconds(parent.gnssTrajectory.front().timestamp-messageTime) > 0.1)
				return;
			guessPose = parent.gnssTrajectory.extrapolate(messageTime);
		}
		else if (messageTime > parent.gnssTrajectory.back().timestamp) {

		}
		else {
			guessPose = parent.gnssTrajectory.interpolate(messageTime);
		}
	}

	else {
		// Guess Pose from last displacement
		Vector3d rot = quaternionToRPY(lastDisplacement.orientation());
		TTransform guessDisplacement = TTransform::from_XYZ_RPY(lastDisplacement.translation(), 0, 0, rot.z());
		guessPose = previous_pose * guessDisplacement;
	}

	GlobalMapperCloud::Ptr output_cloud(new GlobalMapperCloud);
	mNdt.setInputSource(newScan);
	mNdt.align(*output_cloud, guessPose.matrix().cast<float>());
	Pose ndtPose = mNdt.getFinalTransformation().cast<double>();

	// Calculate difference between NDT pose and predicted pose
	double predict_pose_error = (ndtPose.position()-guessPose.position()).norm();

	if (predict_pose_error <= PREDICT_POSE_THRESHOLD) {
		currentPose = ndtPose;
	}
	else {
		currentPose = guessPose;
	}

	// velocity and acceleration not needed

	// XXX: Calculate NDT reliability

	lastDisplacement = previous_pose.inverse() * currentPose;

	previous_pose = currentPose;
	vehicleTrack.push_back(PoseStamped(currentPose, messageTime));
	currentScanId++;

	return;
}

}	// namespace LidarMapper


