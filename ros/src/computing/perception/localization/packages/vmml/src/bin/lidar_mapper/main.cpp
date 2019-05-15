/*
 * main.cpp
 *
 *  Created on: May 13, 2019
 *      Author: sujiwo
 */


#include "LidarMapper.h"


/*
 * Parameter Description
 * 1st: Path to Bag file
 * 2nd: Path to configuration file
 * 3nd: Path to LiDAR calibration file
 */
int main(int argc, char *argv[])
{
	LidarMapper::LidarMapper::createMapFromBag(argv[1], argv[2], argv[3]);
	return 0;
}
