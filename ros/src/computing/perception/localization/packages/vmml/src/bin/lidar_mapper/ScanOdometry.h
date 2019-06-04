/*
 * ScanOdometry.h
 *
 *  Created on: Jun 4, 2019
 *      Author: sujiwo
 */

#ifndef _SCANODOMETRY_H_
#define _SCANODOMETRY_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>

#include "utilities.h"

class LidarMapper;


class ScanOdometry
{
public:
	ScanOdometry(const LidarMapper &parent);
	virtual ~ScanOdometry();

	Pose match(const ptime &stamp, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud);

protected:
	// keyframe parameters
	double scan_delta_trans;  			// minimum distance between scans
	double scan_angle;					//
	double scan_delta_time;				//

	// registration validation by thresholding
	bool transform_thresholding;  //
	double max_acceptable_trans;  //
	double max_acceptable_angle;

	// odometry calculation
	Pose prev_trans;							// previous estimated transform from scan
	Pose keyframe_pose;							// scan pose
	ptime keyframe_stamp;						// scan time
	pcl::PointCloud<pcl::PointXYZI>::ConstPtr keyframe;  // scan point cloud

	//	  pcl::Filter<PointT>::Ptr downsample_filter;
	pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;

};

#endif /* _SCANODOMETRY_H_ */
