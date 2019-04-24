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
#include "datasets/MeidaiBagDataset.h"


class NdtLocalizer2 {
public:

	NdtLocalizer2();

	virtual ~NdtLocalizer2();

	void loadMap (const std::string &filename);

	void putEstimation (const Pose &pEst, const ptime &estTime);

	static void localizeFromBag (LidarScanBag &bagsrc, Trajectory &resultTrack, const Trajectory &gnssTrack, const std::string &pcdMapFile);

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

	pcl_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> mNdt;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcMap = nullptr;

};

#endif /* _NDTLOCALIZER2_H_ */
