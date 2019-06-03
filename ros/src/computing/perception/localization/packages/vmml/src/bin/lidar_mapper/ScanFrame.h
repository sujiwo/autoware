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

#include "utilities.h"


namespace LidarMapper {

struct ScanFrame {
	ScanFrame();
	virtual ~ScanFrame();

	ptime timestamp;
	pcl::PointCloud<pcl::PointXYZI>::ConstPtr scan;
	Pose odometry;
	Pose gnssPose;

	// Put g2o vertex here
};

}

#endif /* _LIDAR_MAPPER_SCANFRAME_H_ */
