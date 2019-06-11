/*
 * LoopDetector.cpp
 *
 *  Created on: Jun 9, 2019
 *      Author: sujiwo
 */

#include <limits>

#include "LoopDetector.h"
#include "LidarMapper.h"
#include "PoseGraph.h"

using namespace std;


typedef pcl::PointXYZI PointT;


namespace LidarMapper {

LoopDetector::LoopDetector(LidarMapper &p):
	parent(p),
	graph(p.graph),
	distanceFromLastEdgeThreshold(p.getParams().min_edge_interval),
	accumDistanceThresh(p.getParams().accum_distance_thresh),
	maxLoopDistance(p.getParams().max_loop_distance)
{
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

		double dist = (kf->node->estimate().translation() - newScanFrame->node->estimate().translation()).norm();
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
	for (const auto &nk: new_keyframes) {
		auto candidates = findCandidates(keyframes, nk);
	}
}


Loop::Ptr
LoopDetector::validate(const std::vector<ScanFrame::Ptr> &candidateList, const ScanFrame::Ptr &newScanFrame)
{
	// Re-read point cloud from bag
	auto cloud2 = parent.lidarBag->getFiltered<PointT>(newScanFrame->bagId);
	matcher.setInputTarget(cloud2);

	pcl::PointCloud<PointT> aligned;
	double best_score = numeric_limits<double>::max();
	ScanFrame::Ptr bestMatch;
	TTransform relativePose;

	for (auto &candidate: candidateList) {
		auto cloud1 = parent.lidarBag->getFiltered<PointT>(candidate->bagId);
		matcher.setInputSource(cloud1);
		Eigen::Matrix4f guess = (newScanFrame->node->estimate().inverse() * candidate->node->estimate()) .matrix().cast<float>();
		matcher.align(aligned, guess);

		cout << '.' << flush;

		double score = matcher.getFitnessScore();
		if (!matcher.hasConverged() or score > best_score)
			continue;

		best_score = score;
		bestMatch = candidate;
		relativePose = matcher.getFinalTransformation();
	}

	return nullptr;
}


} /* namespace LidarMapper */



