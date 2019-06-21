/*
 * GlobalMapper.cpp
 *
 *  Created on: May 16, 2019
 *      Author: sujiwo
 */

#include <cstdio>
#include <pcl/io/pcd_io.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include "LidarMapper.h"


using namespace std;


namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive &ar, LidarMapper::GlobalMapper::ScanProcessLog &log, unsigned int version)
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
		& log.submap_size
		& log.submap_origin_stamp
		& log.submap_origin_pose
		& log.hasScanFrame
		& log.prevScanFrame
		& log.accum_distance
		& log.gnssIsUsed;
}

}
}



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
	ScanProcessLog curResult;
	curResult.sequence_num = scanId;
	curResult.timestamp = messageTime;

	try {
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
			curResult.gnssIsUsed = true;
			throw curResult;
		}

		else if (scanId==parent.generalParams.startId+1) {
			currentPose = current_gnss_pose;
			lastDisplacement = previous_pose.inverse() * currentPose;
			vehicleTrack.push_back(PoseStamped(currentPose, messageTime));
			previous_pose = currentPose;
			curResult.gnssIsUsed = true;
			throw curResult;
		}

		else if (fitness_score>=500.0) {
			guessPose = current_gnss_pose;
			curResult.gnssIsUsed = true;
		}

		else {
			// Guess Pose from last displacement
			Vector3d rot = quaternionToRPY(lastDisplacement.orientation());
			TTransform guessDisplacement = TTransform::from_XYZ_RPY(lastDisplacement.translation(), 0, 0, rot.z());
			guessPose = previous_pose * guessDisplacement;
			curResult.gnssIsUsed = true;
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
		curResult.fitness_score = mNdt.getFitnessScore();
		curResult.transformation_probability = mNdt.getTransformationProbability();
		curResult.num_of_iteration = mNdt.getFinalNumIteration();

		lastDisplacement = previous_pose.inverse() * currentPose;

		previous_pose = currentPose;
		vehicleTrack.push_back(PoseStamped(currentPose, messageTime));
		curResult.poseAtScan = currentPose;

		throw curResult;

	// Logging
	} catch (ScanProcessLog &spl) {
		scanResults.insert(make_pair(scanId, spl));
	}

	return;
}


void
GlobalMapper::saveLog(fstream &output) const
{
	boost::archive::binary_oarchive logger(output);
	logger << scanResults;
}


void
GlobalMapper::readLog(fstream &input)
{
	boost::archive::binary_iarchive logInput(input);
	logInput >> scanResults;
}


}	// namespace LidarMapper


