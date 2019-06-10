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
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
//#include <g2o/edge_se3_plane.hpp>
//#include <g2o/edge_se3_priorxy.hpp>
//#include <g2o/edge_se3_priorxyz.hpp>
//#include <g2o/edge_se3_priorvec.hpp>
//#include <g2o/edge_se3_priorquat.hpp>

#include "ScanFrame.h"
#include "g2o_types/edge_se3_priorxyz.hpp"

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

	inline int size() const
	{ return frameList.size(); }

	bool addScanFrame(ScanFrame::Ptr &f);

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
	g2o::EdgeSE3PriorXYZ* createSE3PriorEdge(
		g2o::VertexSE3* v_se3,
		const Eigen::Vector3d& xyz,
		const Eigen::MatrixXd& information_matrix);
	void addRobustKernel(
		g2o::OptimizableGraph::Edge* edge,
		const std::string &kernel_type,
		double kernel_size);

	Eigen::MatrixXd calculateInformationMatrix
	(const ScanFrame &from, const ScanFrame &to);
};

} /* namespace LidarMapper */

#endif /* _LIDAR_MAPPER_POSEGRAPH_H_ */
