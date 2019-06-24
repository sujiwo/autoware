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
#include <limits>

// Change to whichever version of NDT you like
#include <pclomp/ndt_omp.h>

#include "ScanFrame.h"
#include "utilities.h"


namespace LidarMapper {


class LidarMapper;


struct Loop {
	using Ptr = std::shared_ptr<Loop>;

	Loop(const ScanFrame::Ptr &f1, const ScanFrame::Ptr &f2,
		TTransform relative_pose,
		double fitScore=std::numeric_limits<double>::max()):

		frame1(f1),
		frame2(f2),
		transform2to1(relative_pose),
		fitness_score(fitScore)
	{}

	/*
	 * Notes: frame1 -> new frame
	 *        frame2 -> old frame
	 */
	ScanFrame::Ptr frame1, frame2;
	TTransform transform2to1;

	// We may need to put other information
	// eg. information matrix, fitness score?
	double fitness_score;
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

	// Special matcher for loop detector
	pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> matcher;

	double
		distanceFromLastEdgeThreshold,
		accumDistanceThresh,
		maxLoopDistance;
	double lastAccumDistance = 0.0;

	std::vector<ScanFrame::Ptr>
	findCandidates(const std::vector<ScanFrame::Ptr> &frameList, const ScanFrame::Ptr &newScanFrame) const;

	Loop::Ptr validate(const std::vector<ScanFrame::Ptr> &candidates, const ScanFrame::Ptr &newScanFrame);
};

} /* namespace LidarMapper */

#endif /* _LIDAR_MAPPER_LOOPDETECTOR_H_ */
