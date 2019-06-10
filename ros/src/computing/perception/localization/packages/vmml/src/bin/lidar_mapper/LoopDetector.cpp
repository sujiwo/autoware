/*
 * LoopDetector.cpp
 *
 *  Created on: Jun 9, 2019
 *      Author: sujiwo
 */

#include "LoopDetector.h"
#include "LidarMapper.h"
#include "PoseGraph.h"

using namespace std;


namespace LidarMapper {

LoopDetector::LoopDetector(LidarMapper &p):
	parent(p),
	graph(p.graph)
{
}


LoopDetector::~LoopDetector()
{
}


std::vector<ScanFrame::Ptr>
LoopDetector::findCandidates(const std::vector<ScanFrame::Ptr> &frameList, const ScanFrame::Ptr &newKeyFrame) const
{
	vector<ScanFrame::Ptr> candidates;

	return candidates;
}


Loop::Ptr
LoopDetector::validate(const std::vector<ScanFrame::Ptr> &candidates, const ScanFrame::Ptr &newScanFrame)
{
	return nullptr;
}


} /* namespace LidarMapper */



