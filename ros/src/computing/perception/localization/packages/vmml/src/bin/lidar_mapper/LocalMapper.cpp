/*
 * LocalMapper.cpp
 *
 *  Created on: May 15, 2019
 *      Author: sujiwo
 */


#include "LidarMapper.h"


namespace LidarMapper {


LocalMapper::LocalMapper(const LocalMapper::Param &p):
	param(p)
{
	// Parameter set
	mNdt.setResolution(param.ndt_res);
	mNdt.setStepSize(param.step_size);
	mNdt.setTransformationEpsilon(param.trans_eps);
	mNdt.setMaximumIterations(param.max_iter);

	mVoxelGridFilter.setLeafSize(param.voxel_leaf_size, param.voxel_leaf_size, param.voxel_leaf_size);

}


void
LocalMapper::feed(LidarScanBag::scan_t::ConstPtr &newscan)
{

	// End
	currentScanId += 1;
}


}	// namespace LidarMapper
