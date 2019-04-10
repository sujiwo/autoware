/*
 * LocalizationObserver.cpp
 *
 *  Created on: May 23, 2016
 *      Author: sujiwo
 */

#include "LocalizationObserver.h"
#include <tf/transform_listener.h>
#include <thread>
#include <cmath>
#include <cstdio>


void quaternionToEuler (
	const double &qx, const double &qy, const double &qz, const double &qw,
	double &roll, double &pitch, double &yaw
)
{
	Eigen::Matrix3d rotMat = Eigen::Quaterniond (qw, qx, qy, qz).toRotationMatrix();

	int i = 0, j = 1, k = 2;
	double ax, ay, az;
	double cy = sqrt(pow(rotMat(i, i), 2) + pow(rotMat(j, i), 2));
	if (cy > std::numeric_limits<double>::epsilon()*4) {
		ax = atan2(rotMat(k, j), rotMat(k, k));
		ay = atan2(-rotMat(k, i), cy);
		az = atan2(rotMat(j, i), rotMat(i, i));
	}
	else {
		ax = atan2(-rotMat(j, k), rotMat(j, j));
		ay = atan2(-rotMat(k, i), cy);
		az = 0.0;
	}
	roll = ax, pitch = ay, yaw = az;
}


void eulerToQuaternion (
		const double roll,
		const double pitch,
		const double yaw,
		double &qx, double &qy, double &qz, double &qw)
{
	int i = 0, j = 1, k = 2;

	double
		r = roll / 2.0,
		p = pitch / 2.0,
		y = yaw / 2.0;
    double
		ci = cos(r),
		si = sin(r),
		cj = cos(p),
		sj = sin(p),
		ck = cos(y),
		sk = sin(y),
		cc = ci*ck,
		cs = ci*sk,
		sc = si*ck,
		ss = si*sk;
    qx = cj * sc - sj * cs;
    qy = cj * ss + sj * cc;
    qz = cj * cs - sj * sc;
    qw = cj * cc + sj * ss;
}


LocalizationObserver::LocalizationObserver() :
	number(0)
{
	// TODO Auto-generated constructor stub

}


LocalizationObserver::~LocalizationObserver()
{
	for (tfObserver &obs: observerList) {
		obs.doStop = true;
		obs.proc->join();
		delete (obs.proc);
	}
}


void LocalizationObserver::addTfListener (const string from, const string to)
{
	tfObserver mvListen;
	mvListen.doStop = false;
	mvListen.isOk = false;
	observerList.push_back(mvListen);
	mvListen.proc = new thread(&LocalizationObserver::run, this, from, to, number);
	cout << "Added listener: " << from << " -> " << to << endl;
	number += 1;
}


void LocalizationObserver::collectObservations (vector<tf::Transform> &obs)
{
	observationLock.lock();
	obs.clear();

	for (const tfObserver &observer: observerList) {
		if (observer.isOk==true) {
			tf::Transform rot90 = tf::Transform::getIdentity();
			tf::Quaternion fpq;
			fpq.setRPY(0,0,M_PI/2);
			fpq.normalize();
			rot90.setRotation(fpq);
			obs.push_back(observer.currentPose*rot90);
		}
	}

	observationLock.unlock();
}


void LocalizationObserver::run (const string from, const string to, int pn)
{
	tf::TransformListener thListen;
	tfObserver &observation = observerList[pn];
	observation.doStop = false;

	ros::Rate fps (25);
	while (observation.doStop==false) {
		tf::StampedTransform cTransform;

		try {
			ros::Time t = ros::Time::now();
			thListen.waitForTransform(from, to, t, ros::Duration(0.1));
			thListen.lookupTransform(from, to, t, cTransform);
			{
				observationLock.lock();
				observation.currentPose.setOrigin(cTransform.getOrigin());
				observation.currentPose.setRotation(cTransform.getRotation());
				observation.isOk = true;
				observationLock.unlock();

//				double roll, pitch, yaw;
//				quaternionToEuler(cTransform, roll, pitch, yaw);
//				cout << to << ": " << cTransform.getOrigin().x() << " " << cTransform.getOrigin().y();
//				printf (" Roll: %lf, Pitch: %lf, Yaw: %lf\n", roll, pitch, yaw);

			}
//		} catch (const tf::TransformException &e) {
		} catch (...) {
			observation.isOk = false;
			continue;
		}

		fps.sleep();
	}
}
