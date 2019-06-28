/*
 * RVizConnector.h
 *
 *  Created on: Jun 28, 2019
 *      Author: sujiwo
 */

#ifndef _LIDAR_MAPPER_RVIZCONNECTOR_H_
#define _LIDAR_MAPPER_RVIZCONNECTOR_H_


#include <ros/ros.h>
#include "visualization_msgs/Marker.h"


namespace LidarMapper {

class RVizConnector
{
public:
	RVizConnector(bool enabled=false);
	virtual ~RVizConnector();

	void sendGlobalPose();

	void sendTransformedScan();

protected:
	bool isEnabled;
};

}		// namespace LidarMapper

#endif /* _LIDAR_MAPPER_RVIZCONNECTOR_H_ */
