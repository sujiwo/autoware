/*
 * ScanFrame.h
 *
 *  Created on: Jun 3, 2019
 *      Author: sujiwo
 */

#ifndef _LIDAR_MAPPER_SCANFRAME_H_
#define _LIDAR_MAPPER_SCANFRAME_H_

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <g2o/types/slam3d/vertex_se3.h>

#include "utilities.h"


namespace LidarMapper {

struct ScanFrame {

	typedef std::shared_ptr<ScanFrame> Ptr;

	inline ScanFrame(int64_t i, const ptime &t, const Pose &localPose, const Pose &gps, double ad, double fitness=0.0) :
		bagId(i),
		timestamp(t),
		odometry(localPose),
		gnssPose(gps),
		accum_distance(ad),
		fitnessScore(fitness)
	{}

	inline static Ptr create(int64_t i, const ptime &t, const Pose &localPose, const Pose &gps, double ad, double fitness=0.0)
	{ return Ptr(new ScanFrame(i, t, localPose, gps, ad, fitness)); }

	inline const Pose getPose() const
	{ return Pose::fromIsometry(node->estimate()); }

	ptime timestamp;
//	pcl::PointCloud<pcl::PointXYZI>::ConstPtr scan;
	Pose odometry;				// Supplied by local mapper
	Pose gnssPose;				// Supplied by GNSS
	double accum_distance;		// Supplied by local mapper
	double fitnessScore = 0.0;	// Supplied by local mapper

	// XXX: Put global matching results here also

	// Frame Number in related bag
	int64_t bagId;

	g2o::VertexSE3* node=nullptr;		// node instance

	std::string dump() const;
};

}

#endif /* _LIDAR_MAPPER_SCANFRAME_H_ */
