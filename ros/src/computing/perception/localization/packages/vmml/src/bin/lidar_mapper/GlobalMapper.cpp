/*
 * GlobalMapper.cpp
 *
 *  Created on: May 16, 2019
 *      Author: sujiwo
 */

#include <cstdio>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>

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
		& log.gnssIsUsed
		& log.matchingTime
		& log.currentVelocity;
}

}
}


typedef pcl::Filter<pcl::PointXYZ>::Ptr FilterPtr;


namespace LidarMapper {

const double PREDICT_POSE_THRESHOLD = 2.5;


GlobalMapper::GlobalMapper(LidarMapper &_parent, const GlobalMapper::Param &p) :
	parent(_parent),
	param(p),
	globalMap(new GlobalMapperCloud)
{
	auto iniCfg = parent.getRootConfig();

	// Parameter set
	inipp::extract(iniCfg.sections["Local Mapping"]["max_scan_range"], param.max_scan_range);
	if (param.max_scan_range < parent.localMapperParameters.min_scan_range)
		param.max_scan_range = 100.0;

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

	mNdt.setInputTarget(globalMap);

	octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(param.step_size));
	octree->setInputCloud(globalMap);
	octree->addPointsFromInputCloud();

	cout << "Initial Map loaded" << endl;
}


GlobalMapper::GlobalMapperCloud::Ptr
GlobalMapper::findNearestInMap(const Pose &pt) const
{
	pcl::PointXYZ queryPt(pt.x(), pt.y(), pt.z());
	vector<int> indices;
	vector<float> sqDist;
	int n = octree->radiusSearch(queryPt, param.max_scan_range, indices, sqDist);

	if (n==0)
		return nullptr;
	else {
		GlobalMapperCloud::Ptr inBox(new GlobalMapperCloud(*globalMap, indices));
		return inBox;
	}
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
			throw curResult;

		// see if position is available in GNSS trajectory
		Pose guessPose, currentPose;

		if (scanId==parent.generalParams.startId) {
			currentPose = current_gnss_pose;
			previous_pose = current_gnss_pose;
			vehicleTrack.push_back(PoseStamped(currentPose, messageTime));
			curResult.gnssIsUsed = true;
			curResult.fitness_score = 0.0;
			throw curResult;
		}

		else if (scanId==parent.generalParams.startId+1) {
			currentPose = current_gnss_pose;
			lastDisplacement = previous_pose.inverse() * currentPose;
			vehicleTrack.push_back(PoseStamped(currentPose, messageTime));
			previous_pose = currentPose;
			curResult.gnssIsUsed = true;
			curResult.fitness_score = 0.0;
			throw curResult;
		}

		// XXX: Dubious threshold
		else if (fitness_score>=500.0) {
			guessPose = current_gnss_pose;
			curResult.gnssIsUsed = true;
		}

		else {
			// Guess Pose from last displacement
			Vector3d rot = quaternionToRPY(lastDisplacement.orientation());
			TTransform guessDisplacement = TTransform::from_XYZ_RPY(lastDisplacement.translation(), 0, 0, rot.z());
			guessPose = previous_pose * guessDisplacement;
			curResult.gnssIsUsed = false;
		}

		// XXX: Filtering test
/*
		auto surroundsPcd = findNearestInMap(guessPose);
		pcl::io::savePCDFileBinary("/tmp/box.pcd", *surroundsPcd);
*/

		ptime trun1 = getCurrentTime();
		GlobalMapperCloud::Ptr output_cloud(new GlobalMapperCloud);
		mNdt.setInputSource(newScan);
		mNdt.align(*output_cloud, guessPose.matrix().cast<float>());
		Pose ndtPose = mNdt.getFinalTransformation().cast<double>();
		ptime trun2 = getCurrentTime();
		curResult.matchingTime = trun2 - trun1;

		// Calculate difference between NDT pose and predicted pose
		double predict_pose_error = (ndtPose.position()-guessPose.position()).norm();

		if (predict_pose_error <= PREDICT_POSE_THRESHOLD) {
			currentPose = ndtPose;
			curResult.isValid = true;
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
		// Calculate velocity first
		if (scanId!=parent.generalParams.startId) {
			auto prevLog = getScanLog(scanId-1);
			spl.currentVelocity = Twist(prevLog.poseAtScan, spl.poseAtScan, toSeconds(messageTime-prevLog.timestamp));
		}

		scanResults.insert(make_pair(scanId, spl));
	} catch (std::exception &e) {
		cerr << "Unknown exception: " << e.what() << endl;
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

	// rebuild trajectory
	vehicleTrack.clear();
	for (auto &l: scanResults) {
		PoseStamped p(l.second.poseAtScan, l.second.timestamp);
		vehicleTrack.push_back(p);
	}
}


}	// namespace LidarMapper


