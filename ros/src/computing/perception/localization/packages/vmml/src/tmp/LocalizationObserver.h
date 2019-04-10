/*
 * LocalizationObserver.h
 *
 *  Created on: May 23, 2016
 *      Author: sujiwo
 */

#ifndef _LOCALIZATIONOBSERVER_H_
#define _LOCALIZATIONOBSERVER_H_

#include <thread>
#include <mutex>
#include <iostream>
#include <string>
#include <iostream>
#include <vector>
#include <tf/tf.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>



using namespace std;


struct tfObserver {
	tf::Transform currentPose;
	bool isOk;
	thread *proc;
	volatile bool doStop;
};


void quaternionToEuler (
	const double &qx, const double &qy, const double &qz, const double &qw,
	double &roll, double &pitch, double &yaw);

template <typename typed>
void quaternionToEuler (const Eigen::Quaternion<typed> &quat, double &roll, double &pitch, double &yaw)
{ return quaternionToEuler(quat.x(), quat.y(), quat.z(), quat.w(), roll, pitch, yaw); }

inline void quaternionToEuler (const tf::Transform &tsrc, double &roll, double &pitch, double &yaw)
{
	return quaternionToEuler (
		tsrc.getRotation().x(),
		tsrc.getRotation().y(),
		tsrc.getRotation().z(),
		tsrc.getRotation().w(),
		roll, pitch, yaw);
}


// Vice versa

void eulerToQuaternion (
	const double roll,
	const double pitch,
	const double yaw,
	double &qx, double &qy, double &qz, double &qw);

template <typename t>
void eulerToQuaternion (
	const double roll,
	const double pitch,
	const double yaw,
	Eigen::Quaternion<t> &qm)
{
	return eulerToQuaternion (roll, pitch, yaw, qm.x(), qm.y(), qm.z(), qm.w());
}

inline void eulerToQuaternion (
	const double roll,
	const double pitch,
	const double yaw,
	tf::Transform &tg
)
{
	double qx, qy, qz, qw;
	eulerToQuaternion (roll, pitch, yaw, qx, qy, qz, qw);
	tg.setRotation(tf::Quaternion(qx, qy, qz, qw));
}


class LocalizationObserver
{
public:
	LocalizationObserver();
	virtual ~LocalizationObserver();

	void addTfListener (const string from, const string to);
	void collectObservations (vector<tf::Transform> &obs);

protected:
	mutex observationLock;
	vector<tfObserver> observerList;
	int number;

//	void run (const string from, const string to, tfObserver *observation);
	void run (const string from, const string to, int number);
};

#endif /* _LOCALIZATIONOBSERVER_H_ */
