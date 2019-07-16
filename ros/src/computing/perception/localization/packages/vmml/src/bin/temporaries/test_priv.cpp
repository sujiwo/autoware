/*
 * test_priv.cpp
 *
 *  Created on: Aug 12, 2018
 *      Author: sujiwo
 */


#include <iostream>
#include <string>
#include <utility>
#include <Eigen/Eigen>

#include "ImageDatabase.h"

using namespace std;
using namespace Eigen;


int main(int argc, char *argv[])
{
	ORBVocabulary myVocab;
	myVocab.loadFromTextFile("/tmp/ORBvoc.txt");

	return 0;
}
