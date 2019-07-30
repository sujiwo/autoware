/*
 * TrajectoryWithCovariance.h
 *
 *  Created on: Jul 29, 2019
 *      Author: sujiwo
 */

#ifndef _LIDAR_MAPPER_TRAJECTORYWITHCOVARIANCE_H_
#define _LIDAR_MAPPER_TRAJECTORYWITHCOVARIANCE_H_

#include "Trajectory.h"


namespace LidarMapper {

class TrajectoryWithCovariance : public Trajectory
{

public:
	TrajectoryWithCovariance (const Trajectory &);

	typedef Trajectory Parent;
	typedef Eigen::Matrix<double,6,6> CovarMat;

	void push_back(const PoseStamped &);

	inline CovarMat getCovarianceAt (const int idx) const
	{ return covariances.at(idx); }

protected:
	std::vector<CovarMat> covariances;
	Twist currentVelocity;
};

} /* namespace LidarMapper */

#endif /* _LIDAR_MAPPER_TRAJECTORYWITHCOVARIANCE_H_ */
