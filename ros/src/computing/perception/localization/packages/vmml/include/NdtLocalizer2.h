/*
 * NdtLocalizer2.h
 *
 *  Created on: Apr 24, 2019
 *      Author: sujiwo
 */

#ifndef _NDTLOCALIZER2_H_
#define _NDTLOCALIZER2_H_

#include <memory>
#include <string>

#include <pcl_omp_registration/ndt.h>

#include "utilities.h"
#include "Trajectory.h"
#include "ParticleFilter.h"
#include "datasets/MeidaiBagDataset.h"


class NdtLocalizer2 : public PF::VehicleBase<Pose, Pose, TTransform> {
public:

	NdtLocalizer2(const Pose &initialEstimation);

	virtual ~NdtLocalizer2();

	void loadMap (const std::string &filename);

	void putEstimation (const Pose &pEst, const ptime &estTime);

	static void localizeFromBag (LidarScanBag &bagsrc, Trajectory &resultTrack, const Trajectory &gnssTrack, const std::string &pcdMapFile);

	static TTransform getTransform(const LidarScanBag::scan_t::ConstPtr &scan1, const LidarScanBag::scan_t::ConstPtr &scan2, bool &isConverged);

	struct Parameters
	{
		int maximum_iterations = 30;
		float ndt_resolution = 1.0;
		double step_size = 0.1;
		double transformation_epsilon = 0.01;
		double max_correspondence_distance = 1.0;
		double euclidean_fitness_epsilon = 0.1;
		double ransac_outlier_rejection_threshold = 1.0;
		Pose initialGuess = Pose::Identity();

		enum {
			Linear, Quadratic, Zero
		} offsetMode;

	} mParams;

protected:

	const Pose initPose;

	pcl_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> mNdt;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcMap = nullptr;

	PF::ParticleFilter<Pose, Pose, TTransform> pFilter;

	ptime lastLocalizationTime;

	// For particle filter
	virtual Pose initializeParticleState() const;
	virtual Pose motionModel(const Pose &vstate, const TTransform &ctrl) const;
	virtual double measurementModel(const Pose &state, const vector<Pose> &observations) const;

	PoseStamped summarizePf(const ptime &t, const Quaterniond &orientationSet);
};

#endif /* _NDTLOCALIZER2_H_ */
