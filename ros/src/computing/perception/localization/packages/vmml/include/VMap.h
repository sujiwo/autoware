/*
 * VMap.h
 *
 *  Created on: Jun 13, 2018
 *      Author: sujiwo
 */

#ifndef VMAP_H_
#define VMAP_H_

#include <vector>
#include <string>
#include <set>
#include <map>
#include <Eigen/Eigen>
#include <limits>
#include <mutex>

#include <boost/noncopyable.hpp>
#include <boost/serialization/serialization.hpp>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/graph/adjacency_list.hpp>

#include "utilities.h"
#include "CameraPinholeParams.h"
#include "datasets/GenericDataset.h"


class KeyFrame;
class MapPoint;
class ImageDatabase;


#define MAX_ORB_POINTS_IN_FRAME 9000


enum FeatureDetectorT {
	ORB,
	AKAZE,
	BRISK
};

enum DescriptorMatcherT {
	BruteForce
};

class Frame;

class Trajectory;

struct kpidField : std::vector<bool>
{
	kpidField() : std::vector<bool>() {};

	kpidField(const int sz, const bool init=false):
		std::vector<bool> (sz, init) {}

	void invert();

	uint countPositive() const;

	// A Mask is used to compare keypoints in 1 against 2
	static cv::Mat createMask (const kpidField &fld1, const kpidField &fld2);
};


struct MapFileHeader {
	FeatureDetectorT _featureDt;
	DescriptorMatcherT _descrptMt;
	uint64
		numOfKeyFrame,
		numOfMapPoint;
};


class VMap

{
public:

	VMap ();
	VMap (const cv::Mat &_mask, cv::Ptr<cv::FeatureDetector> fdetect, cv::Ptr<cv::DescriptorMatcher> dmatcher);
	VMap (const cv::Mat &_mask, FeatureDetectorT _fdetector, DescriptorMatcherT _dmatcher);
	virtual ~VMap();

	void reset();

	int addCameraParameter (const CameraPinholeParams &vscamIntr);

	void setMask(cv::Mat maskSrc);

	const cv::Mat getMask() const
	{ return mask; }

	inline const CameraPinholeParams getCameraParameter(const int cameraId)
	{ return cameraList[cameraId]; }

	int getNumberOfCameras () const
	{ return cameraList.size(); }

	pcl::PointCloud<pcl::PointXYZ>::Ptr dumpPointCloudFromMapPoints ();

	kfid createKeyFrame (
		const cv::Mat &imgSrc,
		const Eigen::Vector3d &p, const Eigen::Quaterniond &o,
		const int cameraId,
		KeyFrame **ptr=NULL,
		dataItemId setSourceId=std::numeric_limits<dataItemId>::max(),
		ptime tm=boost::posix_time::not_a_date_time);

	mpid createMapPoint (
		const Eigen::Vector3d &p,
		MapPoint **ptr=NULL);

	inline KeyFrame* getKeyFrameById (const kfid &i) const
	{ return keyframeInvIdx.at(i); }

	inline MapPoint* getMapPointById (const mpid &i) const
	{ return mappointInvIdx.at(i); }

	std::vector<kfid> getKeyFrameList() const;
	std::vector<mpid> getMapPointList() const;

	KeyFrame* keyframe (const kfid &i)
	{ return keyframeInvIdx.at(i); }

	const KeyFrame* keyframe (const kfid &i) const
	{ return keyframeInvIdx.at(i); }

	MapPoint* mappoint (const mpid &i)
	{ return mappointInvIdx.at(i); }

	const MapPoint* mappoint (const mpid &i) const
	{ return mappointInvIdx.at(i); }

	void estimateStructure (const kfid &keyFrame1, const kfid &keyFrame2, double translationHint=-1.0);

	void estimateAndTrack (const kfid &kfid1, const kfid &kfid2, const double metricDisposition=1.0);

	std::vector<kfid> allKeyFrames () const;
	std::vector<mpid> allMapPoints () const;

	int numOfKeyFrames() const
	{ return keyframeInvIdx.size(); }

	int numOfMapPoints() const
	{ return mappointInvIdx.size(); }

	std::vector<mpid> getVisibleMapPoints (const kfid &kf) const;

	inline int countRelatedKeyFrames(const mpid &i) const
	{ return pointAppearances.at(i).size(); }

	inline std::set<kfid> getRelatedKeyFrames (const mpid &i) const
	{ return pointAppearances.at(i); }

	inline kpid getKeyPointId (const kfid k, const mpid p) const
	{ return framePoints.at(k).at(p); }

	inline mpid getMapPointByKeypoint (const kfid k, const kpid p)
	{ return framePointsInv.at(k).at(p); }

	// XXX: Causes SIGSEGV when framePoints are empty
	std::map<mpid,kpid> allMapPointsAtKeyFrame(const kfid f) const;

	bool save (const std::string &path);
	bool load (const std::string &path);

	std::vector<std::pair<Eigen::Vector3d,Eigen::Quaterniond> >
		dumpCameraPoses () const;

	void dumpCameraPoses (Trajectory &track) const;

	kpidField makeField (const kfid &kf);
	kpidField visibleField (const kfid &kf);

	ImageDatabase* getImageDB() const
	{ return imageDB; }

	kfid search (const Frame &f) const;

	static cv::Ptr<cv::FeatureDetector> createFeatureDetector(FeatureDetectorT t);
	static cv::Ptr<cv::DescriptorMatcher> createDescriptorMatcher(DescriptorMatcherT t);

	cv::Ptr<cv::FeatureDetector> getFeatureDetector () const
	{ return featureDetector; }

	cv::Ptr<cv::DescriptorMatcher> getDescriptorMatcher () const
	{ return descriptorMatcher; }

	std::vector<CameraPinholeParams> getCameraParameters() const
	{ return cameraList; }

	std::vector<kfid> getOrderedRelatedKeyFramesFrom (const kfid k, int howMany=-1) const;

	std::vector<kfid> getKeyFramesComeInto (const kfid kTarget) const;

	void trackMapPoints (const kfid kf1, const kfid kf2);

	bool removeMapPoint (const mpid &i);
	bool removeMapPointsBatch (const vector<mpid> &mplist);

	typedef std::map<std::string,std::string> mapKeyValueInfo;

	inline void setInfo (const std::string &key, const std::string &value)
	{ keyValueInfo[key] = value; };

	inline const std::string getInfo (const std::string &key) const
	{ return keyValueInfo.at(key); }

	inline const mapKeyValueInfo& getAllInfo() const
	{ return keyValueInfo; }

	void fixFramePointsInv ();

protected:

	friend class KeyFrame;

	friend class Matcher;

	cv::Mat vocabulary;

//	std::vector<KeyFrame*> keyframeList;
//	std::vector<MapPoint*> mapPointList;

	std::mutex *keyframeInvIdx_mtx;
	std::map<kfid, KeyFrame*> keyframeInvIdx;

	std::mutex *mappointInvIdx_mtx;
	std::map<mpid, MapPoint*> mappointInvIdx;

	// Relationship between MapPoint and KeyFrames

	// Maps MapPoint to set of KeyFrames in which it appears
	std::map<mpid, std::set<kfid> > pointAppearances;

	// List all map points that appears in a keyframe with
	// their ID of KeyPoint (ie. their projections)
	// KeyFrame -> (MapPoint, KeyPoint)
	std::map<kfid,
		std::map<mpid, kpid> > framePoints;

	// List of all map points with inverted of the above
	// It will be useful eg. to check projection (see Localizer.cpp)
	// KeyFrame -> (KeyPoint, MapPoint)
	std::map<kfid,
		std::map<kpid, mpid> > framePointsInv;

	// 2D image stuff
	cv::Mat mask;
	cv::Ptr<cv::FeatureDetector> featureDetector;
	cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
	FeatureDetectorT curDetector;
	DescriptorMatcherT curDescMatcher;

	std::vector<CameraPinholeParams> cameraList;

	ImageDatabase *imageDB;

	typedef boost::property<boost::edge_weight_t, int> EdgeWeightProperty;
	typedef boost::adjacency_list<boost::listS, boost::vecS, boost::bidirectionalS, boost::no_property, EdgeWeightProperty> KeyFrameGraph;
	typedef boost::graph_traits<KeyFrameGraph>::edge_iterator edge_iterator;

	KeyFrameGraph covisibility;
	std::map<kfid,KeyFrameGraph::vertex_descriptor> kfVtxMap;
	std::map<KeyFrameGraph::vertex_descriptor,kfid> kfVtxInvMap;
	std::mutex covisibilityGraphMtx;

	// Information about this map
	mapKeyValueInfo keyValueInfo;

	void updateCovisibilityGraph(const kfid k);

	static std::vector<double> mScaleFactors;
	static std::vector<double> mLevelSigma2;

};

#endif /* VMAP_H_ */
