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

#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/edge_se3_prior.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>


#include "Trajectory.h"
#include "ScanFrame.h"
#include "LoopDetector.h"
//#include "g2o_types/edge_se3_priorxyz.hpp"

namespace LidarMapper {


class LidarMapper;


/*
 * Abstraction of G2O Optimization graph
 */
class PoseGraph {

friend class LidarMapper;

public:

	typedef std::shared_ptr<PoseGraph> Ptr;

	PoseGraph(LidarMapper &p);
	virtual ~PoseGraph();

	void clear();

	void optimize(int numOfIteration);

	inline int size() const
	{ return frameList.size(); }

	bool addScanFrame(ScanFrame::Ptr &f);

	void addGlobalPose (ScanFrame::Ptr &f, const Pose &gpose);

	const std::vector<ScanFrame::Ptr>& getFrameList() const
	{ return frameList; }

	void handleLoop(Loop::Ptr &loop);

	const Trajectory dumpTrajectory() const;

protected:
	LidarMapper &parent;

	g2o::VertexSE3* anchorNode=nullptr;
	g2o::EdgeSE3* anchorEdge=nullptr;

	std::vector<ScanFrame::Ptr> frameList;

	std::unique_ptr<g2o::SparseOptimizer> graph=nullptr;

	g2o::RobustKernelFactory* robust_kernel_factory;

	g2o::VertexSE3* createSE3Node(const TTransform &tf);

	g2o::EdgeSE3* createSE3Edge(
		g2o::VertexSE3* v1, g2o::VertexSE3* v2,
		const TTransform &relativePose,
		const Eigen::MatrixXd &informationMatrix);

	g2o::EdgeSE3Prior* createSE3PriorEdge(
		g2o::VertexSE3* v_se3,
		const TTransform& pose,
		const Eigen::MatrixXd& information_matrix);

	void addRobustKernel(
		g2o::OptimizableGraph::Edge* edge,
		const std::string &kernel_type,
		double kernel_size);

	g2o::EdgeSE3Prior* createGnssPrior(g2o::VertexSE3* v_se3, const ptime &t);

	Eigen::MatrixXd calculateInformationMatrix();

	Eigen::MatrixXd calculateInformationMatrix
	(const Loop &loop);

	// Information matrix generator
	double
		const_stddev_x,
		const_stddev_q;
	double
		gnss_stddev_horizontal,
		gnss_stddev_vertical,
		gnss_stddev_angular;
};

} /* namespace LidarMapper */

#endif /* _LIDAR_MAPPER_POSEGRAPH_H_ */
