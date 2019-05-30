/*
 * GlobalMapper.cpp
 *
 *  Created on: May 16, 2019
 *      Author: sujiwo
 */

#include <cstdio>
#include <pcl/io/pcd_io.h>

#include "LidarMapper.h"


using namespace std;


namespace LidarMapper {

const double PREDICT_POSE_THRESHOLD = 2.5;


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


void GlobalMapper::feed(GlobalMapperCloud::ConstPtr &newScan, const ptime &messageTime, int scanId)
{
	currentScanId = scanId;
	current_gnss_pose = parent.getGnssPose(messageTime);

	// Do nothing when prior map is not available
	if (globalMap->empty()==true)
		return;

	// see if position is available in GNSS trajectory
	Pose guessPose, currentPose;

	if (scanId==parent.generalParams.startId) {
		currentPose = current_gnss_pose;
		previous_pose = current_gnss_pose;
		vehicleTrack.push_back(PoseStamped(currentPose, messageTime));
		return;
	}

	else if (scanId==parent.generalParams.startId+1) {
		currentPose = current_gnss_pose;
		lastDisplacement = previous_pose.inverse() * currentPose;
		vehicleTrack.push_back(PoseStamped(currentPose, messageTime));
		previous_pose = currentPose;
		return;
	}

	else if (fitness_score>=500.0) {
		guessPose = current_gnss_pose;
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

	// Calculate NDT reliability
	fitness_score = mNdt.getFitnessScore();
	transformation_probability = mNdt.getTransformationProbability();
	num_iterations = mNdt.getFinalNumIteration();

	// Logging
	printf("F: %f; T: %f; I: %d\n", fitness_score, transformation_probability, num_iterations);

	lastDisplacement = previous_pose.inverse() * currentPose;

	previous_pose = currentPose;
	vehicleTrack.push_back(PoseStamped(currentPose, messageTime));

	return;
}



}	// namespace LidarMapper


