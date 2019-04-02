/*
 * main.cpp
 *
 *  Created on: Aug 9, 2018
 *      Author: sujiwo
 */


#include <iostream>
#include <string>
#include <boost/filesystem.hpp>

#include <ros/package.h>

#include <QApplication>
#include "DatasetBrowser.h"
#include "datasets/OxfordDataset.h"
#include "datasets/MeidaiBagDataset.h"

using namespace std;


const string oxfordModelDir = "/home/sujiwo/Sources/robotcar-dataset-sdk/models";


int main (int argc, char *argv[])
{
	cout << fixed << setprecision(6);

	QApplication mainApp(argc, argv);
	DatasetBrowser dbs;

	boost::filesystem::path datasetPath(argv[1]);
//	GenericDataset *dataset;

	if (boost::filesystem::is_directory(datasetPath)) {
		auto o_dataset = OxfordDataset::load(datasetPath.string(), oxfordModelDir);
		o_dataset->setZoomRatio(0.5);
		dbs.changeDataset(o_dataset, DatasetBrowser::OxfordType);
	}

	else if (datasetPath.extension()==".bag") {
		auto b_dataset = MeidaiBagDataset::load(datasetPath.string());
		dbs.changeDataset(b_dataset, DatasetBrowser::MeidaiType);
	}

	else {
		cerr << "Unsupported dataset type" << endl;
	}

	dbs.show();

	return mainApp.exec();
}
