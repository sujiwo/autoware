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
	uint32_t i = size()-1;
	// find loops

	// Insert to tree
//	positionLookups.
}
