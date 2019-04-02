/*
 * Viewer.cpp
 *
 *  Created on: Jul 10, 2018
 *      Author: sujiwo
 */

#include <map>
#include "Viewer.h"
#include "VMap.h"
#include "utilities.h"
#include "KeyFrame.h"


using namespace std;


static string wndName ("Viewer");
static cv::Vec3b kpColor(255,0,0);
static cv::Vec3b lineColor(0, 255, 0);


Viewer::Viewer (GenericDataset::ConstPtr genset) :
	dataset(genset),
	cMap(NULL)
{
	cv::namedWindow(wndName);
}


Viewer::~Viewer()
{
	cv::destroyWindow(wndName);
}


void
Viewer::setMap (VMap *m)
{ cMap = m; }


vector<cv::KeyPoint>
collectAllKeypoints (const KeyFrame *kf, const vector<kpid> &allKpIds)
{
	vector<cv::KeyPoint> allKp;
	for (auto &i: allKpIds) {
		allKp.push_back(kf->keypoint(i));
	}
	return allKp;
}


void
Viewer::update (int dataItemId, kfid frameId)
{
	assert (cMap != NULL);
	KeyFrame *kf = cMap->keyframe(frameId);
	cv::Mat imagebuf = dataset->get(dataItemId)->getImage();

	// Draw visible map points and their optical flow
	const map<mpid,kpid> visibleMp = cMap->allMapPointsAtKeyFrame(frameId);
	if (visibleMp.size() != 0) {
		vector<kpid> allMpKpIds = allValues(visibleMp);
		auto allKeypoints = collectAllKeypoints(kf, allMpKpIds);
		cv::drawKeypoints(imagebuf, allKeypoints, imagebuf, kpColor);

		if (kf->previousKeyframe != std::numeric_limits<kfid>::max()) {

			KeyFrame *anchor = cMap->keyframe(kf->previousKeyframe);
//			vector<cv::KeyPoint> anchorKeypoints;
			for (auto kp: allMpKpIds) {
				mpid mp = cMap->getMapPointByKeypoint(frameId, kp);
				try {
					kpid kpidAnchor = cMap->getKeyPointId(kf->previousKeyframe, mp);
					cv::KeyPoint kpAnchor = anchor->keypoint(kpidAnchor);
					cv::KeyPoint kpCurrent = kf->keypoint(kp);
					cv::line(imagebuf, kpCurrent.pt, kpAnchor.pt, lineColor);
//					anchorKeypoints.push_back(anchor->keypoint(kpAnchor));
				} catch (out_of_range &e) {
					continue;
				}
			}
		}
	}

	cv::imshow(wndName, imagebuf);
	cv::waitKey(1);
}
