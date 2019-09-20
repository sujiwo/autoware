/*
 * VMapBuilder.cpp
 *
 *  Created on: Jul 18, 2019
 *      Author: sujiwo
 */

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>

#include "VMapBuilder.h"
#include "LidarMapper.h"
#include "inipp.h"

using namespace boost::filesystem;

namespace LidarMapper {


VMapBuilder::VMapBuilder(rosbag::Bag &bagfd, const std::string &imageTopic, LidarMapper &lm) :
	parent(&lm)
{
	imageBag.reset(new RandomAccessBag(bagfd, imageTopic));
	visMap.reset(new VMap);

	// Camera Configuration
	auto conf = lm.getRootConfig();
	inipp::extract(conf.sections["Camera"]["fx"], monoCam.fx);
	inipp::extract(conf.sections["Camera"]["fy"], monoCam.fy);
	inipp::extract(conf.sections["Camera"]["cx"], monoCam.cx);
	inipp::extract(conf.sections["Camera"]["cy"], monoCam.cy);
	auto img0 = imageBag->at<sensor_msgs::Image>(0);
	monoCam.width = img0->width;
	monoCam.height = img0->height;

	visMap->addCameraParameter(monoCam);

	path maskPath = lm.getWorkDir() / "mask.png";
	if (exists(maskPath)) {
		cv::Mat maskImg = cv::imread(maskPath.string(), cv::IMREAD_GRAYSCALE);
		visMap->setMask(maskImg);
	}
}


VMapBuilder::~VMapBuilder()
{
}


bool
VMapBuilder::feed(const ScanFrame &lframe)
{
	auto image = getImage(lframe);

	return true;
}


/*
 * We suppose that this function can be called only from LocalMapper
 */
bool
VMapBuilder::feed(pcl::PointCloud<pcl::PointXYZI>::ConstPtr scan, const ptime &tstamp)
{
	auto image = getImage(tstamp);
	if (hasStarted==false) {

	}

	else {

	}

	return true;
}


cv::Mat
VMapBuilder::getImage(const ScanFrame &lframe)
{
	return getImage(lframe.timestamp);
}


cv::Mat
VMapBuilder::getImage(const ptime &timestamp)
{
	// Find corresponding image frame
	auto idx = imageBag->getPositionAtTime(ros::Time::fromBoost(timestamp));
	auto _imgs = imageBag->at<sensor_msgs::Image>(idx);
	auto gp = cv_bridge::toCvCopy(*_imgs, sensor_msgs::image_encodings::BGR8);
	return gp->image;

}

} /* namespace LidarMapper */
