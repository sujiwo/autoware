/*
 * CameraPinholeParams.h
 *
 *  Created on: Oct 16, 2018
 *      Author: sujiwo
 */

#ifndef _CAMERAPINHOLEPARAMS_H_
#define _CAMERAPINHOLEPARAMS_H_


#include <Eigen/Core>
#include <boost/serialization/serialization.hpp>


struct CameraPinholeParams
{
	double
		fx=0, fy=0,
		cx=0, cy=0;
	int width, height;
	// XXX: Distortion parameters

	CameraPinholeParams(
		double _fx, double _fy,
		double _cx, double _cy,
		int _w, int _h):
		fx(_fx), fy(_fy),
		cx(_cx), cy(_cy),
		width(_w), height(_h)
	{}

	Eigen::Matrix<double,3,4> toMatrix() const;

	Eigen::Matrix3d toMatrix3() const;

	cv::Mat toCvMat() const;

	CameraPinholeParams():
		width(-1), height(-1)
	{}

	template<class Archive>
	void serialize(Archive &ar, const unsigned int file_version)
	{ ar & fx & fy & cx & cy & width & height; }

	static CameraPinholeParams
	loadCameraParamsFromFile(const std::string &f);

	CameraPinholeParams
	operator* (const float f) const;

	// Field of Views, in radian
	float getHorizontalFoV () const;
	float getVerticalFoV () const;

	inline double f() const
	{ return (fx+fy)/2; }

	inline Eigen::Vector2d principalPoints() const
	{ return Eigen::Vector2d(cx, cy); }
};



#endif /* _CAMERAPINHOLEPARAMS_H_ */
