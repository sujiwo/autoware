/*
 * PoseGraph.cpp
 *
 *  Created on: Jun 8, 2019
 *      Author: sujiwo
 */

#include <iostream>

#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include "PoseGraph.h"
#include "LidarMapper.h"

using namespace std;


G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

/*
namespace g2o {
  G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
}
*/




namespace LidarMapper {

PoseGraph::PoseGraph(LidarMapper &p) :
	parent(p)
{
	auto iniCfg = parent.getRootConfig();
	inipp::extract(iniCfg.sections["Scan Matching"]["const_stddev_x"], const_stddev_x);
	inipp::extract(iniCfg.sections["Scan Matching"]["const_stddev_q"], const_stddev_q);

	inipp::extract(iniCfg.sections["GNSS"]["stddev_horizontal"], gnss_stddev_horizontal);
	inipp::extract(iniCfg.sections["GNSS"]["stddev_vertical"], gnss_stddev_vertical);
	inipp::extract(iniCfg.sections["GNSS"]["stddev_angular"], gnss_stddev_angular);

	graph.reset(new g2o::SparseOptimizer);
	g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
	g2o::OptimizationAlgorithmProperty solver_property;
	g2o::OptimizationAlgorithm* solver = solver_factory->construct("lm_var_cholmod", solver_property);
	graph->setAlgorithm(solver);

	g2o::ParameterSE3Offset *param = new g2o::ParameterSE3Offset;
	param->setOffset(g2o::Isometry3::Identity());
	param->setId(0);
	graph->addParameter(param);

	robust_kernel_factory = g2o::RobustKernelFactory::instance();
}


PoseGraph::~PoseGraph()
{}


bool
PoseGraph::addScanFrame(ScanFrame::Ptr &f)
{
	// First vertex ?
	if (frameList.empty()) {
		anchorNode = createSE3Node(TTransform::Identity());
		f->node = anchorNode;
		auto edge1 = createGnssPrior(f->node,  f->timestamp);
		addRobustKernel(edge1, "Huber", 0.1);
		frameList.push_back(f);
		return true;
	}

	// New vertex
	f->node = createSE3Node(f->odometry);

	// Edge between consecutive frames
	cout << "Adding frame" << endl;
	const auto &prevFrame = frameList.back();
	auto relativeMovement = prevFrame->odometry.inverse() * f->odometry;
	auto informMat = calculateInformationMatrix();
	auto edge2 = createSE3Edge(prevFrame->node, f->node, relativeMovement, informMat);

	// Add an Unary Edge with GNSS constraint
/*
	auto gnssPose = parent.getGnssPose(f->timestamp);
	auto edge1 = createGnssPrior(f->node, f->timestamp);
	addRobustKernel(edge1, "Huber", 0.1);
*/
	frameList.push_back(f);

	return true;
}


void
PoseGraph::addGlobalPose (ScanFrame::Ptr &f, const Pose &gpose)
{

}


g2o::VertexSE3*
PoseGraph::createSE3Node(const TTransform &tf)
{
	g2o::VertexSE3* vertex(new g2o::VertexSE3);
	vertex->setId(static_cast<int>(graph->vertices().size())+1);
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


/*
 * Generate fixed information matrix
 */
Eigen::MatrixXd
PoseGraph::calculateInformationMatrix()
{
	Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6);
	information.topLeftCorner(3, 3).array() /= const_stddev_x;
	information.bottomRightCorner(3, 3).array() /= const_stddev_q;

	return information;
}


/*
 * Generate information matrix for loop
 */
Eigen::MatrixXd
PoseGraph::calculateInformationMatrix(const Loop &loop)
{
	Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6);

	return information;
}


void
PoseGraph::addRobustKernel(g2o::OptimizableGraph::Edge* edge, const std::string &kernel_type, double kernel_size)
{
	g2o::RobustKernel *k = robust_kernel_factory->construct(kernel_type);
	k->setDelta(kernel_size);
	edge->setRobustKernel(k);
}


g2o::EdgeSE3Prior*
PoseGraph::createSE3PriorEdge(g2o::VertexSE3* v_se3, const TTransform& pose, const Eigen::MatrixXd& information_matrix)
{
	g2o::EdgeSE3Prior* edge(new g2o::EdgeSE3Prior);
	edge->setMeasurement(pose.toIsometry());
	edge->setInformation(information_matrix);
	edge->vertices()[0] = v_se3;
	edge->setParameterId(0, 0);
	graph->addEdge(edge);
	return edge;
}


g2o::EdgeSE3Prior*
PoseGraph::createGnssPrior(g2o::VertexSE3* v_se3, const ptime &t)
{
	auto gPose = parent.getGnssPose(t);

	g2o::Matrix6 information_matrix = g2o::Matrix6::Identity(6, 6);
	information_matrix.block<2, 2>(0, 0) /= gnss_stddev_horizontal;
	information_matrix(2, 2) /= gnss_stddev_vertical;
	information_matrix.bottomRightCorner(3, 3).array() /= gnss_stddev_angular;

	return createSE3PriorEdge(v_se3, gPose, information_matrix);
}


void
PoseGraph::handleLoop(Loop::Ptr &loop)
{
	auto infMat = calculateInformationMatrix();
	auto edge = createSE3Edge(loop->frame2->node, loop->frame1->node, loop->transform2to1, infMat);
	addRobustKernel(edge, "Huber", 1.0);
}


void
PoseGraph::optimize(int numOfIteration)
{
	if (graph->edges().size() < 10)
		return;

	cout << "Graph Optimization... " << flush;

	graph->initializeOptimization();
	graph->computeInitialGuess();
	graph->computeActiveErrors();
	graph->setVerbose(true);

	double chi2_1 = graph->chi2();

	auto t1 = getCurrentTime();
	int iterations = graph->optimize(numOfIteration);
	auto t2 = getCurrentTime();

	double sec = toSeconds(t2-t1);
	cout << "Done: " << sec << " seconds" << endl;
}


const Trajectory
PoseGraph::dumpTrajectory() const
{
	Trajectory frames;

	for (int i=0; i<frameList.size(); ++i) {
		auto frame = frameList[i];
		Pose framePose = frame->getPose();
		frames.push_back(PoseStamped(framePose, frame->timestamp));
	}

	return frames;
}

} /* namespace LidarMapper */
