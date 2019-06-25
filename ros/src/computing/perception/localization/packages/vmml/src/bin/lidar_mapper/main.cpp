/*
 * main.cpp
 *
 *  Created on: May 13, 2019
 *      Author: sujiwo
 */

#include <iostream>
#include <vector>
#include <memory>

#include <pcl/registration/registration.h>
#include "LidarMapper.h"


/*
 * Parameter Description
 * 1st: Path to Bag file
 * 2nd: Path to working directory
 */
int main(int argc, char *argv[])
{
	try {
		LidarMapper::LidarMapper::createMapFromBag(argv[1], argv[2]);
	} catch (std::exception &e) {
		std::cerr << "Error: " << e.what() << "\n";
	}
	return 0;
}
