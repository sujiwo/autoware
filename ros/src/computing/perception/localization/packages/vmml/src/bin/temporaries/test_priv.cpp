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
	MeidaiBagDataset::Ptr meidaiDs = MeidaiBagDataset::load("/media/sujiwo/ssd/campus_loop/campus_loop-1.bag");

	auto lidarBag = meidaiDs->getLidarScanBag();
	auto gnssTrajectory = meidaiDs->getGnssTrajectory();
	Trajectory ndtTrajectory;

	NdtLocalizer2::localizeFromBag(*lidarBag, ndtTrajectory, gnssTrajectory, "/home/sujiwo/Data/NagoyaUniversityMap/bin_meidai_ndmap.pcd");

	return 0;
}
