/*
 * LoopDetector.cpp
 *
 *  Created on: Jun 9, 2019
 *      Author: sujiwo
 */

#include <limits>

#include "LoopDetector.h"
#include "LidarMapper.h"

using namespace std;


typedef pcl::PointXYZI PointT;


const double fitness_score_threshold = 0.5;


namespace LidarMapper {

LoopDetector::LoopDetector(LidarMapper &p):
	parent(p)
{
	auto parentCfg = parent.getRootConfig();
	inipp::extract(parentCfg.sections["General"]["min_edge_interval"], distanceFromLastEdgeThreshold);
	inipp::extract(parentCfg.sections["General"]["accum_distance_thresh"], accumDistanceThresh);
	inipp::extract(parentCfg.sections["General"]["max_loop_distance"], maxLoopDistance);
}


LoopDetector::~LoopDetector()
{
}


std::vector<ScanFrame::Ptr>
LoopDetector::findCandidates(const std::vector<ScanFrame::Ptr> &frameList, const ScanFrame::Ptr &newScanFrame) const
{
	vector<ScanFrame::Ptr> candidates;

	// Too close to last loop
	if (newScanFrame->accum_distance - lastAccumDistance < distanceFromLastEdgeThreshold)
		return candidates;

	for (const auto &kf: frameList) {
		if (newScanFrame->accum_distance - kf->accum_distance < accumDistanceThresh) {
			continue;
		}

		// Take GNSS position
		Vector3d
			kfPos = kf->gnssPose.position(),
			newFramePos = newScanFrame->gnssPose.position();
		double dist = (kfPos - newFramePos).norm();
		if (dist > maxLoopDistance) {
			continue;
		}

		candidates.push_back(kf);
	}

	return candidates;
}


std::vector<Loop::Ptr>
LoopDetector::detect(
const std::vector<ScanFrame::Ptr> &keyframes,
const std::deque<ScanFrame::Ptr> &new_keyframes)
{
	vector<Loop::Ptr> detectedLoops;

	for (const auto &nk: new_keyframes) {
		auto candidates = findCandidates(keyframes, nk);
		auto loop = validate(candidates, nk);
		if (loop) {
			detectedLoops.push_back(loop);
		}
	}

	return detectedLoops;
}


Loop::Ptr
LoopDetector::validate(const std::vector<ScanFrame::Ptr> &candidateList, const ScanFrame::Ptr &newScanFrame)
{
	// Re-read point cloud from bag
	auto cloud1 = parent.lidarBag->getFiltered<PointT>(newScanFrame->bagId);
	matcher.setInputTarget(cloud1);

	pcl::PointCloud<PointT> aligned;
	double best_score = numeric_limits<double>::max();
	ScanFrame::Ptr bestMatch;
	TTransform relativePose;

	for (auto &candidate: candidateList) {
		auto cloud2 = parent.lidarBag->getFiltered<PointT>(candidate->bagId);
		matcher.setInputSource(cloud2);
//		Eigen::Matrix4f guess = (newScanFrame->node->estimate().inverse() * candidate->node->estimate()) .matrix().cast<float>();
		Eigen::Matrix4f guess = (newScanFrame->gnssPose.inverse() * candidate->gnssPose) .matrix().cast<float>();

		matcher.align(aligned, guess);

		cout << '.' << flush;

		double score = matcher.getFitnessScore();
		if (!matcher.hasConverged() or score > best_score)
			continue;

		best_score = score;
		bestMatch = candidate;
		relativePose = matcher.getFinalTransformation();
	}

	if (best_score > fitness_score_threshold) {
//		cout << "Loop not found" << endl;
		return nullptr;
	}
	else {
		cout << "Loop found: " << newScanFrame->bagId << "->" << bestMatch->bagId << endl;
		lastAccumDistance = newScanFrame->accum_distance;
	}

	return make_shared<Loop>(newScanFrame, bestMatch, relativePose, best_score);
}


} /* namespace LidarMapper */



