/*
 * TrajectoryWithCovariance.cpp
 *
 *  Created on: Jul 29, 2019
 *      Author: sujiwo
 */

#include "TrajectoryWithCovariance.h"

namespace LidarMapper {


TrajectoryWithCovariance::TrajectoryWithCovariance (const Trajectory &src)
{
	for (int i=0; i<src.size(); ++i) {
		auto px = src.at(i);
		push_back(px);
	}
}


void
TrajectoryWithCovariance::push_back(const PoseStamped &px)
{
	Parent::push_back(px);

	CovarMat cov;
	if (size()==1) {
		cov = CovarMat::Identity();
		covariances.push_back(cov);
	}

	else if (size()==2) {
		cov = CovarMat::Identity();
		currentVelocity = Twist(px, Parent::at(0));
		covariances.push_back(cov);
	}

	else {
		// estimate desired pose
		PoseStamped lastGood = this->at(size()-2);
		auto disp = currentVelocity.displacement(px.timestamp - lastGood.timestamp);
		const PoseStamped estimated = lastGood * disp;

		// calculate covariance

		// calculate velocity
		// xxx: unfinished
	}
}


} /* namespace LidarMapper */
