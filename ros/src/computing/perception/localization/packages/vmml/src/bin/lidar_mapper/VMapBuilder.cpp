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
#include "inipp.h"


using namespace std;
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
	inipp::extract(conf.sections["Camera"]["image_rescale"], imageRescale);
	auto img0 = imageBag->at<sensor_msgs::Image>(0);
	monoCam.width = img0->width;
	monoCam.height = img0->height;
	monoCam = monoCam*imageRescale;

	// Transformation from lidar to camera
	Vector3d tt, rpy;
	inipp::extract(conf.sections["Lidar to Camera"]["x"], tt.x());
	inipp::extract(conf.sections["Lidar to Camera"]["y"], tt.y());
	inipp::extract(conf.sections["Lidar to Camera"]["z"], tt.z());
	inipp::extract(conf.sections["Lidar to Camera"]["roll"], rpy.x());
	inipp::extract(conf.sections["Lidar to Camera"]["pitch"], rpy.y());
	inipp::extract(conf.sections["Lidar to Camera"]["yaw"], rpy.z());
	lidarToCamera = TTransform::from_XYZ_RPY(tt, rpy.x(), rpy.y(), rpy.z());

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

	if (initialized==false) {

		// We have no keyframe yet
		if (pivot==numeric_limits<kfid>::max()) {

		}

		else {

			initialized = true;
		}
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
