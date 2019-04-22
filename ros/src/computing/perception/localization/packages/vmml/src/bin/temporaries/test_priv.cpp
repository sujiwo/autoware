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
#include "utilities.h"


using namespace std;
using namespace Eigen;


int main(int argc, char *argv[])
{
	MeidaiBagDataset::Ptr meidaiDs = MeidaiBagDataset::load("/media/sujiwo/ssd/campus_loop/campus_loop-1.bag");
	auto gnssTrajectory = meidaiDs->getGnssTrajectory();
	auto frame1340 = meidaiDs->get(1340);
	auto p1 = gnssTrajectory.find_lower_bound(frame1340->getTimestamp());
	auto tw1 = gnssTrajectory.getVelocityAt(p1);
	auto d = tw1.displacement(0.1);

	return 0;
}
