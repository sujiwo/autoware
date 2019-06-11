/*
 * PoseGraph.cpp
 *
 *  Created on: Jun 8, 2019
 *      Author: sujiwo
 */

#include "PoseGraph.h"
#include "LidarMapper.h"


namespace LidarMapper {

PoseGraph::PoseGraph(LidarMapper &p) :
	parent(p)
{
	graph.reset(new g2o::SparseOptimizer);
	g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
	g2o::OptimizationAlgorithmProperty solver_property;
	g2o::OptimizationAlgorithm* solver = solver_factory->construct("lm_var_cholmod", solver_property);
	graph->setAlgorithm(solver);

	robust_kernel_factory = g2o::RobustKernelFactory::instance();
}


PoseGraph::~PoseGraph()
{}


bool
PoseGraph::addScanFrame(ScanFrame::Ptr &f)
{
	// New vertex
	f->node = createSE3Node(f->odometry);

	// First vertex ?
	if (frameList.empty()) {
		anchorNode = createSE3Node(TTransform::Identity());
		anchorNode->setFixed(true);
		anchorEdge = createSE3Edge(anchorNode, f->node, TTransform::Identity(), Eigen::MatrixXd::Identity(6, 6));
	}

	// Edge between consecutive frames
	const auto &prevFrame = frameList.back();
	auto relativeMovement = prevFrame->odometry.inverse() * f->odometry;
	auto informMat = calculateInformationMatrix(*prevFrame, *f);
	auto edge = createSE3Edge(prevFrame->node, f->node, relativeMovement, informMat);

	// Add an Unary Edge with GNSS constraint
	auto gnssPose = parent.getGnssPose(f->timestamp);
	Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
	information_matrix.block<2, 2>(0, 0) /= parent.getParams().gnss_stddev_horizontal;
	information_matrix(2, 2) /= parent.getParams().gnss_stddev_vertical;
	createSE3PriorEdge(f->node, gnssPose.position(), information_matrix);

	frameList.push_back(f);

	return true;
}


g2o::VertexSE3*
PoseGraph::createSE3Node(const TTransform &tf)
{
	g2o::VertexSE3* vertex(new g2o::VertexSE3);
	vertex->setId(static_cast<int>(graph->vertices().size()));
	vertex->setEstimate(tf.toIsometry());
	graph->addVertex(vertex);
	return vertex;
}


g2o::EdgeSE3*
PoseGraph::createSE3Edge(
	g2o::VertexSE3* from, g2o::VertexSE3* to,
	const TTransform &relativePose,
	const Eigen::MatrixXd &informationMatrix)
{
	g2o::EdgeSE3* edge(new g2o::EdgeSE3());
	edge->setMeasurement(relativePose.toIsometry());
	edge->setInformation(informationMatrix);
	edge->vertices()[0] = from;
	edge->vertices()[1] = to;
	graph->addEdge(edge);

	return edge;
}


Eigen::MatrixXd
PoseGraph::calculateInformationMatrix
(const ScanFrame &from, const ScanFrame &to)
{
	Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6);

	// XXX: Unfinished

	return information;
}


void
PoseGraph::addRobustKernel(g2o::OptimizableGraph::Edge* edge, const std::string &kernel_type, double kernel_size)
{
	g2o::RobustKernel *k = robust_kernel_factory->construct(kernel_type);
	k->setDelta(kernel_size);
	edge->setRobustKernel(k);
}


g2o::EdgeSE3PriorXYZ*
PoseGraph::createSE3PriorEdge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix)
{
	g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ);
	edge->setMeasurement(xyz);
	edge->setInformation(information_matrix);
	edge->vertices()[0] = v_se3;
	graph->addEdge(edge);
	return edge;
}

} /* namespace LidarMapper */