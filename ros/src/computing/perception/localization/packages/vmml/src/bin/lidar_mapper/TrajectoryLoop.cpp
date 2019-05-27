/*
 * TrajectoryLoop.cpp
 *
 *  Created on: May 27, 2019
 *      Author: sujiwo
 */


#include "TrajectoryLoop.h"

using namespace std;


void
TrajectoryLoop::push_back(const PoseStamped &pt)
{
	Trajectory::push_back(pt);

	// find loops

	// Insert to tree
}
