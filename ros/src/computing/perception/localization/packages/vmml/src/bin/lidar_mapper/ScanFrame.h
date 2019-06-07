/*
 * ScanFrame.h
 *
 *  Created on: Jun 3, 2019
 *      Author: sujiwo
 */

#ifndef _LIDAR_MAPPER_SCANFRAME_H_
#define _LIDAR_MAPPER_SCANFRAME_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <g2o/types/slam3d/vertex_se3.h>

#include "utilities.h"


namespace LidarMapper {

struct ScanFrame {

	typedef std::shared_ptr<ScanFrame> Ptr;

	inline ScanFrame(int64_t i, const ptime &t, const Pose &localPose, const Pose &gps, double ad) :
		bagId(i),
		timestamp(t),
		odometry(localPose),
		gnssPose(gps),
		accum_distance(ad)
	{}

	ptime timestamp;
//	pcl::PointCloud<pcl::PointXYZI>::ConstPtr scan;
	Pose odometry;				// Supplied by local mapper
	Pose gnssPose;				// Supplied by GNSS
	double accum_distance;		// Supplied by local mapper

	// Frame Number in related bag
	int64_t bagId;

	g2o::VertexSE3* node=nullptr;		// node instance
};

}

#endif /* _LIDAR_MAPPER_SCANFRAME_H_ */
