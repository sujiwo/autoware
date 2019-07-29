/*
 * TrajectoryWithCovariance.cpp
 *
 *  Created on: Jul 29, 2019
 *      Author: sujiwo
 */

#include "TrajectoryWithCovariance.h"

namespace LidarMapper {

void
TrajectoryWithCovariance::push_back(const PoseStamped &p)
{
	Parent::push_back(p);

	CovarMat cov;
	if (size()==1) {
		cov = CovarMat::Identity();
		covariances.push_back(cov);
	}

	else if (size()==2) {
		cov = CovarMat::Identity();
		currentVelocity = Twist(p, Parent::at(0));
		covariances.push_back(cov);
	}

	else {
		currentVelocity = Twist(p, Parent::at(size()-2));
	}
}


} /* namespace LidarMapper */
