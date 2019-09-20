/*
 * VMapBuilder.h
 *
 *  Created on: Jul 18, 2019
 *      Author: sujiwo
 */

#ifndef _VMAPBUILDER_H_
#define _VMAPBUILDER_H_

#include <memory>
#include <string>
#include <rosbag/bag.h>
#include <RandomAccessBag.h>
#include "VMap.h"
#include "ScanFrame.h"


namespace LidarMapper {

class LidarMapper;


class VMapBuilder {
public:
	VMapBuilder(rosbag::Bag &bagfd, const std::string &imageTopic, LidarMapper &lmapper);
	virtual ~VMapBuilder();

	bool feed(const ScanFrame &lframe);
	bool feed(pcl::PointCloud<pcl::PointXYZI>::ConstPtr scan, const ptime &tstamp);

protected:
	std::unique_ptr<VMap> visMap;
	RandomAccessBag::Ptr imageBag;
	LidarMapper *parent;

	CameraPinholeParams monoCam;
	cv::Mat getImage(const ScanFrame &lframe);
	cv::Mat getImage(const ptime &timestamp);

	kfid pivot;
	bool hasStarted = false;
};

} /* namespace LidarMapper */

#endif /* VMML_SRC_BIN_LIDAR_MAPPER_VMAPBUILDER_H_ */
