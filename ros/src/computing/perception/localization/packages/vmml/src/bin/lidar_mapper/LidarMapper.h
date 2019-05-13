/*
 * LidarMapper.h
 *
 *  Created on: May 13, 2019
 *      Author: sujiwo
 */

#ifndef _LIDARMAPPER_H_
#define _LIDARMAPPER_H_

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "utilities.h"
#include "datasets/MeidaiBagDataset.h"


namespace LidarMapper {


class LocalMapper {
public:

struct Param {
	double
		ndt_res,
		step_size,
		trans_eps;
	int
		max_iter;
	double
		voxel_leaf_size,
		min_scan_range,
		min_add_scan_shift,
		max_submap_size;
};

LocalMapper(const Param &p);
void feed();

protected:

};	// LidarMapper::LocalMapper


class GlobalMapper {
public:

struct Param {
	double
		ndt_res,
		step_size,
		trans_eps;
	int
		max_iter,
		queue_size;
};

GlobalMapper(const Param &p);
void loadMap(const std::string &point_cloud_map);
void feed();

protected:

};	// LidarMapper::GlobalMapper



class LidarMapper {
public:
	LidarMapper(const GlobalMapper::Param&, const LocalMapper::Param&);
	virtual ~LidarMapper();

	static void createMapFromBag(const std::string &bagpath);

protected:
	const GlobalMapper::Param globalMapperParameters;
	const LocalMapper::Param localMapperParameters;


};

} // LidarMapper

#endif /* _LIDARMAPPER_H_ */
