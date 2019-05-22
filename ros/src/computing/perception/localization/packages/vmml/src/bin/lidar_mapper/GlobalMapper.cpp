/*
 * GlobalMapper.cpp
 *
 *  Created on: May 16, 2019
 *      Author: sujiwo
 */

#include "LidarMapper.h"


namespace LidarMapper {


GlobalMapper::GlobalMapper(LidarMapper &_parent, const GlobalMapper::Param &p) :
	parent(_parent),
	param(p)
{

}


}	// namespace LidarMapper


