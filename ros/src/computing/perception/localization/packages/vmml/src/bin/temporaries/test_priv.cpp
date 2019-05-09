/*
 * test_priv.cpp
 *
 *  Created on: Aug 12, 2018
 *      Author: sujiwo
 */


#include <iostream>
#include <string>
#include <utility>
#include <Eigen/Eigen>

#include "datasets/MeidaiBagDataset.h"
#include "NdtLocalizer2.h"
#include "utilities.h"


using namespace std;
using namespace Eigen;


int main(int argc, char *argv[])
{
	MeidaiBagDataset::Ptr meidaiDs = MeidaiBagDataset::load("/media/sujiwo/ssd/sample-mapping-tiny.bag");

	auto lidarBag = meidaiDs->getLidarScanBag();
	auto gnssTrajectory = meidaiDs->getGnssTrajectory();
	Trajectory ndtTrajectory;

	NdtLocalizer2::localizeFromBag(*lidarBag, ndtTrajectory, gnssTrajectory, "/home/sujiwo/Data/NagoyaUniversityMap/bin_meidai_ndmap.pcd");

	// XXX: Do something with generated trajectory
	ndtTrajectory.dump("/tmp/ndt_localization.csv");

	return 0;
}
