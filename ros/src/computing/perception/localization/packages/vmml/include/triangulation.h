/*
 * triangulation.h
 *
 *  Created on: Jun 14, 2018
 *      Author: sujiwo
 */

#ifndef TRIANGULATION_H_
#define TRIANGULATION_H_

#include <vector>

#include <Eigen/Eigen>
#include "BaseFrame.h"
#include "Matcher.h"


typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;


// `Optimal' triangulation
bool Triangulate(
	const Matrix3x4d& pose1,
	const Matrix3x4d& pose2,
	const Eigen::Vector2d& point1,
	const Eigen::Vector2d& point2,
	Eigen::Vector4d &triangulated_point);


// Triangulates 2 posed views
bool TriangulateDLT(
	const Matrix3x4d& pose1,
	const Matrix3x4d& pose2,
	const Eigen::Vector2d& point1,
	const Eigen::Vector2d& point2,
	Eigen::Vector4d &triangulated_point);

bool TriangulateCV(
/*
		const Matrix3x4d& pose1,
		const Matrix3x4d& pose2,
		const Eigen::Vector2d& point1,
		const Eigen::Vector2d& point2,
		Eigen::Vector4d &triangulated_point
*/
	const BaseFrame &F1, const BaseFrame &F2,
	const std::vector<Matcher::KpPair> &featurePairs
);

#endif /* TRIANGULATION_H_ */
