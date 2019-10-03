/*
 * ImageLidarFrame.cpp
 *
 *  Created on: Sep 24, 2019
 *      Author: sujiwo
 */

#include <pcl/kdtree/kdtree_flann.h>

#include "ImageLidarFrame.h"


using namespace std;


namespace LidarMapper {


ImageLidarFrame::ImageLidarFrame(
	const cv::Mat &imgSrc,
	cv::Mat &mask,
	cv::Ptr<cv::FeatureDetector> fdetector,
	const CameraPinholeParams *cameraIntr,
	dataItemId imageSrcId, dataItemId lidarSrcId,
	LidarT::Ptr lidarScn) :

		KeyFrame(
			imgSrc,
			Eigen::Vector3d::Identity(), Eigen::Quaterniond::Identity(),
			mask,
			fdetector,
			cameraIntr,
			0),
		lidarScan(lidarScn),
		imageBagId(imageSrcId),
		lidarBagId(lidarSrcId)
{
}


ImageLidarFrame::~ImageLidarFrame() {
	// TODO Auto-generated destructor stub
}


ImageLidarFrame::Ptr
ImageLidarFrame::create(
	RandomAccessBag::Ptr &imageBag,
	LidarT::Ptr lidarScan,
	const ptime lidarTs,
	cv::Mat &mask,
	cv::Ptr<cv::FeatureDetector> fdetector,
	const CameraPinholeParams *cameraIntrinsicParams,
	dataItemId lidarBagId)
{
	// Find corresponding image frame
	auto idx = imageBag->getPositionAtTime(ros::Time::fromBoost(lidarTs));
	auto _imgs = imageBag->at<sensor_msgs::Image>(idx);
	auto gp = cv_bridge::toCvCopy(*_imgs, sensor_msgs::image_encodings::BGR8);

	Ptr retv(new ImageLidarFrame(gp->image, mask, fdetector, cameraIntrinsicParams, idx, lidarBagId, lidarScan));
	retv->imageTimestamp = _imgs->header.stamp.toBoost();
	retv->lidarTimestamp = lidarTs;

	return retv;
}


/*
 * Associate image features with depth from Lidar
 */
void
ImageLidarFrame::buildFeaturePairs(const TTransform &lidarToCameraTransform)
{
	auto lidarProjections = BaseFrame::projectLidarScan(*lidarScan, lidarToCameraTransform, cameraParam);
	pcl::KdTreeFLANN<pcl::PointXY> flannel;

	pcl::PointCloud<pcl::PointXY>::Ptr cloudProjs(new pcl::PointCloud<pcl::PointXY>);
	cloudProjs->reserve(lidarProjections.size());
	for (auto &pt: lidarProjections) {
		cloudProjs->push_back(pcl::PointXY{pt.x, pt.y});
	}
	flannel.setInputCloud(cloudProjs);

	vector<int> index1(1);
	vector<float> dist1(1);
	for (auto &kp: fKeypoints) {
		// Unfinished
		pcl::PointXY pt{pt.x, pt.y};
		index1.clear();
		dist1.clear();
		if (flannel.nearestKSearch(pt, 1, index1, dist1) > 0) {

		}
	}
}

} /* namespace LidarMapper */
