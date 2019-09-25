/*
 * ImageLidarFrame.h
 *
 *  Created on: Sep 24, 2019
 *      Author: sujiwo
 */

#ifndef _LIDAR_MAPPER_IMAGELIDARFRAME_H_
#define _LIDAR_MAPPER_IMAGELIDARFRAME_H_

#include <memory>
#include <vector>
#include <map>

#include <RandomAccessBag.h>
#include "KeyFrame.h"
#include "VMapBuilder.h"
#include "utilities.h"


namespace LidarMapper {

class ImageLidarFrame : public KeyFrame {
public:

	typedef std::shared_ptr<ImageLidarFrame> Ptr;
	typedef pcl::PointCloud<pcl::PointXYZI> LidarT;

	virtual ~ImageLidarFrame();

	static Ptr
	create(
		RandomAccessBag::Ptr &bag,
		LidarT::Ptr lidarScan,
		const ptime lidarTs,
		cv::Mat &mask,
		cv::Ptr<cv::FeatureDetector> fdetector,
		const CameraPinholeParams *cameraIntrinsicParams,
		dataItemId lidarBagId
		);

	void buildFeaturePairs(const TTransform &lidarToCameraTransform);

protected:
	ImageLidarFrame(
		const cv::Mat &imgSrc,
		cv::Mat &mask,
		cv::Ptr<cv::FeatureDetector> fdetector,
		const CameraPinholeParams *cameraIntr,
		dataItemId imageBagId, dataItemId lidarBagId,
		LidarT::Ptr lidarScn);

	LidarT::Ptr lidarScan;

	int64 imageBagId, lidarBagId;

	// Differentiate the timestamps, could be useful in the future
	ptime imageTimestamp, lidarTimestamp;

	struct ImageLidarPair {
		uint32_t imageFeatureId;
		uint32_t pcdId;
		Eigen::Vector3d positionInCam;
	};

	std::map<uint32_t, ImageLidarPair> featurePairs;
};

} /* namespace LidarMapper */

#endif /* _LIDAR_MAPPER_IMAGELIDARFRAME_H_ */
