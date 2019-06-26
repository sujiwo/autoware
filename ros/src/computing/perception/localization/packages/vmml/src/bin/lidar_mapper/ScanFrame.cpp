/*
 * ScanFrame.cpp
 *
 *  Created on: Jun 3, 2019
 *      Author: sujiwo
 */

#include <sstream>

#include "ScanFrame.h"

using namespace std;


namespace LidarMapper {


string ScanFrame::dump() const
{
	stringstream ss;
	Pose g = getPose();

	ss << bagId << ' '
		<< g.position().x() << " "
		<< g.position().y() << " "
		<< g.position().z() << " "
		<< g.orientation().x() << ' '
		<< g.orientation().y() << ' '
		<< g.orientation().z() << ' '
		<< g.orientation().w()
	;
	return ss.str();
}


} // namespace LidarMapper
