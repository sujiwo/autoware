/*
 * PoseGraph.h
 *
 *  Created on: Jun 8, 2019
 *      Author: sujiwo
 */

#ifndef _LIDAR_MAPPER_POSEGRAPH_H_
#define _LIDAR_MAPPER_POSEGRAPH_H_

#include <memory>
#include <vector>
#include <deque>

#include <g2o/core/solver.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>

#include "ScanFrame.h"


namespace LidarMapper {


class LidarMapper;


class PoseGraph {

friend class LidarMapper;

public:

	typedef std::shared_ptr<PoseGraph> Ptr;

	PoseGraph(LidarMapper &p);
	virtual ~PoseGraph();

	void clear();

	void optimize();

protected:
	LidarMapper &parent;

	g2o::VertexSE3* anchorNode;

	std::vector<ScanFrame::Ptr> frameList;
};

} /* namespace LidarMapper */

#endif /* _LIDAR_MAPPER_POSEGRAPH_H_ */
