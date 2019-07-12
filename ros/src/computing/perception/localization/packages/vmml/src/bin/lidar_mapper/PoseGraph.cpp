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

#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>

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

	inipp::extract(iniCfg.sections["Scan Matching"]["min_stddev_x"], min_stddev_x);
	inipp::extract(iniCfg.sections["Scan Matching"]["max_stddev_x"], max_stddev_x);
	inipp::extract(iniCfg.sections["Scan Matching"]["min_stddev_q"], min_stddev_q);
	inipp::extract(iniCfg.sections["Scan Matching"]["max_stddev_q"], max_stddev_q);
	inipp::extract(iniCfg.sections["Scan Matching"]["fitness_score_max"], fitness_score_max);
	inipp::extract(iniCfg.sections["Scan Matching"]["var_gain"], var_gain);

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
		anchorNode = createSE3Node(f->gnssPose);
		f->node = anchorNode;
		auto edge1 = createGnssPrior(f->node,  f->timestamp);
		addRobustKernel(edge1, "Huber", 0.1);
		frameList.push_back(f);
		return true;
	}

	// New vertex
	f->node = createSE3Node(f->odometry);

	// Edge between consecutive frames
	const auto &prevFrame = frameList.back();
	auto relativeMovement = prevFrame->odometry.inverse() * f->odometry;
	auto informMat = calculateInformationMatrix();
	auto edge2 = createSE3Edge(prevFrame->node, f->node, relativeMovement, informMat);

	frameList.push_back(f);

	return true;
}


void
PoseGraph::addGlobalPose (ScanFrame::Ptr &f, const Pose &gpose)
{
/*
	auto vtx = createSE3Node(gpose);
	vtx->setFixed(true);
	f->node->setFixed(false);
	auto infMat = calculateInformationMatrix();
	auto relativeMovement = f->odometry.inverse() * gpose;
	auto edge = createSE3Edge(f->node, vtx, relativeMovement, infMat);
*/
	auto infMat = calculateInformationMatrix();
	auto edge = createSE3PriorEdge(f->node, gpose, infMat);
	addRobustKernel(edge, "Huber", 0.1);
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
	double min_var_x = std::pow(min_stddev_x, 2);
	double max_var_x = std::pow(max_stddev_x, 2);
	double min_var_q = std::pow(min_stddev_q, 2);
	double max_var_q = std::pow(max_stddev_q, 2);

	float w_x = weight(var_gain, fitness_score_max, min_var_x, max_var_x, loop.fitness_score);
	float w_q = weight(var_gain, fitness_score_max, min_var_q, max_var_q, loop.fitness_score);
	information.topLeftCorner(3, 3).array() /= w_x;
	information.bottomRightCorner(3, 3).array() /= w_q;

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


pcl::PointCloud<ResultMapPointT>::Ptr
PoseGraph::createPointCloud ()
{
	pcl::PointCloud<ResultMapPointT>::Ptr newMap(new pcl::PointCloud<ResultMapPointT>);
	pcl::PointCloud<ResultMapPointT>::Ptr submap(new pcl::PointCloud<ResultMapPointT>);
	int64 submapId = parent.localMapperProc->getScanLog(frameList.front()->bagId).submap_id;

	for (auto frame: frameList) {
		auto bagId = frame->bagId;
		auto frameSubmapId = parent.localMapperProc->getScanLog(bagId).submap_id;

		auto frameScan = parent.getLidarBag()->getFiltered<ResultMapPointT>(bagId);
		TTransform scanTrf = frame->getPose();
		pcl::PointCloud<ResultMapPointT> transformedScan;
		pcl::transformPointCloud(*frameScan, transformedScan, scanTrf.matrix().cast<float>());

		if (frameSubmapId==submapId) {
		}

		else {

			// Filter the submap
			pcl::PointCloud<ResultMapPointT>::Ptr filteredGridCLoud(new pcl::PointCloud<ResultMapPointT>);
			pcl::octree::OctreePointCloud<ResultMapPointT> octree(0.05);
			octree.setInputCloud(submap);
			octree.addPointsFromInputCloud();
			octree.getOccupiedVoxelCenters(filteredGridCLoud->points);

			auto currentSubmapPath = parent.workDir/"submaps";
			currentSubmapPath /= std::to_string(submapId)+".pcd";

			pcl::io::savePCDFileBinaryCompressed(currentSubmapPath.string(), *filteredGridCLoud);
			cout << "Outputting " << boost::filesystem::basename(currentSubmapPath) << endl;
/*
			pcl::VoxelGrid<ResultMapPointT> voxel_grid_filter;
			voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);
			voxel_grid_filter.setInputCloud(submap);
			voxel_grid_filter.filter(*filteredGridCLoud);
*/

			*newMap += *filteredGridCLoud;
			submap->clear();
			submapId = frameSubmapId;
		}

		*submap += transformedScan;
	}

	return newMap;
}


double
PoseGraph::weight(double gain, double maxFitnessScore, double Ymin, double Ymax, double fitnessScore)
{
	double Y = (1.0 - std::exp(-gain*fitnessScore)) / (1.0 - std::exp(-gain*maxFitnessScore));
	return Ymin + Y*(Ymax - Ymin);
}


void
PoseGraph::frameLogsDump() const
{
	auto frameDumpPath = parent.workDir / "frames.csv";
	std::fstream fd(frameDumpPath.string(), std::fstream::ios_base::trunc|std::fstream::ios_base::out);

	for (auto &f: frameList) {
		auto s = f->dump();
		fd << s << endl;
	}

	fd.close();
}

} /* namespace LidarMapper */
