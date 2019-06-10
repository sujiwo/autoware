/*
 * LoopDetector.h
 *
 *  Created on: Jun 9, 2019
 *      Author: sujiwo
 */

#ifndef _LIDAR_MAPPER_LOOPDETECTOR_H_
#define _LIDAR_MAPPER_LOOPDETECTOR_H_

#include <vector>
#include <memory>
#include <deque>

#include "ScanFrame.h"
#include "PoseGraph.h"
#include "utilities.h"


namespace LidarMapper {


class LidarMapper;


struct Loop {
	using Ptr = std::shared_ptr<Loop>;

	Loop(const ScanFrame::Ptr &f1, const ScanFrame::Ptr &f2, TTransform relative_pose):
		frame1(f1), frame2(f2), transform(relative_pose)
	{}

	ScanFrame::Ptr frame1, frame2;
	TTransform transform;
};


class LoopDetector {

public:
	typedef std::shared_ptr<LoopDetector> Ptr;

	LoopDetector(LidarMapper &p);
	virtual ~LoopDetector();

	std::vector<Loop::Ptr> detect(
		const std::vector<ScanFrame::Ptr> &keyframes,
		const std::deque<ScanFrame::Ptr> &new_keyframes);

protected:
	LidarMapper &parent;
	PoseGraph::Ptr graph;

	std::vector<ScanFrame::Ptr>
	findCandidates(const std::vector<ScanFrame::Ptr> &frameList, const ScanFrame::Ptr &newScanFrame) const;

	Loop::Ptr validate(const std::vector<ScanFrame::Ptr> &candidates, const ScanFrame::Ptr &newScanFrame);
};

} /* namespace LidarMapper */

#endif /* _LIDAR_MAPPER_LOOPDETECTOR_H_ */
