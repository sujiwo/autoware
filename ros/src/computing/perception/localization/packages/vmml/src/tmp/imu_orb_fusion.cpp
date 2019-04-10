/*
 * imu_orb_fusion.cpp
 *
 *  Created on: May 21, 2016
 *      Author: sujiwo
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <thread>
#include <mutex>
#include "test_tools/IMUPacket.h"
#include "ParticleFilter.h"
#include "LocalizationObserver.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <cmath>


#define DEFAULT_NUM_PARTICLE 500
#define InitialParticleRandomizer 2.0


using namespace PF;


struct ImuState {
	Eigen::Vector3d position;
//	Eigen::Quaterniond orientation;
//	Eigen::Matrix<double, 6, 6> pcovariance;
//
//	Eigen::Vector3d twist_linear;
//	Eigen::Vector3d twist_angular;
//	Eigen::Matrix<double, 6, 6> tcovariance;
	double yaw;

	ImuState () :
		position (Eigen::Vector3d::Zero()),
		yaw (0.0)
	{
//		orientation = Eigen::Quaterniond::Identity();
//		pcovariance = Eigen::Matrix<double,6,6>::Identity();
//		twist_linear = Eigen::Vector3d::Zero();
//		twist_angular = Eigen::Vector3d::Zero();
//		tcovariance = Eigen::Matrix<double,6,6>::Identity();
	}
};


struct ImuMotionCtrl {
	double wx, wy, wz;
	short int wheel;
	double timestamp;
};


static const string
	odomParentFrameId = "world",
	odomFrameId = "CameraEye";


void convert (const tf::Transform &pose, Eigen::Vector3d &position, Eigen::Quaterniond &orientation)
{
	position.x() = pose.getOrigin().x();
	position.y() = pose.getOrigin().y();
	position.z() = pose.getOrigin().z();
	orientation.x() = pose.getRotation().x();
	orientation.y() = pose.getRotation().y();
	orientation.z() = pose.getRotation().z();
	orientation.w() = pose.getRotation().w();
}


void convert (const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, tf::Transform &resPose)
{
	resPose.setOrigin(
		tf::Vector3(
			position.x(),
			position.y(),
			position.z()));
	resPose.setRotation(
		tf::Quaternion(
			orientation.x(),
			orientation.y(),
			orientation.z(),
			orientation.w()));
}


//void quaternionToEuler (const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw)
//{
//
//}



class ImuFusion :
	public VehicleBase <ImuState, tf::Transform, ImuMotionCtrl>,
	public LocalizationObserver
{
public:

	ImuFusion () :
		filter (DEFAULT_NUM_PARTICLE, *this),
		positionFound (false),
		prevTime (0), prevCount (0),
		statePtrList(DEFAULT_NUM_PARTICLE)
	{}


	ImuState initializeParticleState() const
	{
		ImuState cst;
		double roll, pitch, yaw;
		cst.position.x() = initialPose.getOrigin().x();
		cst.position.y() = initialPose.getOrigin().y();
		cst.position.z() = initialPose.getOrigin().z();
		quaternionToEuler (initialPose, roll, pitch, yaw);
//		convert (initialPose, cst.position, cst.orientation);

		// Randomize
		cst.position.x() += PF::nrand(InitialParticleRandomizer);
		cst.position.y() += PF::nrand(InitialParticleRandomizer);
		cst.position.z() += PF::nrand(InitialParticleRandomizer);

		return cst;
	}


	ImuState motionModel (const ImuState &vstate, const ImuMotionCtrl &ctrl) const
	{
		double v;
		double dt = ctrl.timestamp - prevTime;
		short diff = ctrl.wheel - prevCount;

		/*
		 * Why these numbers are not used directly ?
		 */
		if (dt > 1e-4)
			v = (75.0 / 3.e5) * diff / dt;
		else v = 0.0;
		v *= 2.94;
		double wx = (0.03125 * M_PI/180.0) * (short)ctrl.wx;
		double wy = -(0.03125 * M_PI/180.0) * (short)ctrl.wy;
		double wz = (0.03125 * M_PI/180.0) * (short)ctrl.wz;
		wy -= wyOffset;

		ImuState nstate = calcOdometry(vstate, v, wy, dt);
		return nstate;
	}


	double measurementModel (const ImuState &vstate, const vector<tf::Transform> &observations) const
	{
		vector<double> obsWeights;

		for (const auto &pose: observations) {

			double wo,
				xt = pose.getOrigin().x(),
				yt = pose.getOrigin().y(),
				xs = vstate.position.x(),
				ys = vstate.position.y();

				wo = exp (-(pow(xt-xs,2) / (2*orbError*orbError) +
							pow(yt-ys,2) / (2*orbError*orbError)
					));

			obsWeights.push_back(wo);
		}

		double w = *std::max_element(obsWeights.begin(), obsWeights.end());
		return max (w, 0.01);
	}


	void imuCallback (const test_tools::IMUPacket::ConstPtr &imuMsg)
	{
		return;
		vector<tf::Transform> observationList;
		collectObservations (observationList);

		ImuMotionCtrl motion;
		motion.wx = imuMsg->wx;
		motion.wy = imuMsg->wy;
		motion.wz = imuMsg->wz;
		motion.wheel = imuMsg->wheel;
		motion.timestamp = imuMsg->header.stamp.toSec();

		if (positionFound==false) {
			if (observationList.size() != 0) {
				initialPose = observationList[0];
				filter.initializeParticles();
				positionFound = true;
				prevTime = imuMsg->header.stamp.toSec();
				prevCount = imuMsg->wheel;
				cout << "Position initialized" << endl;
			}

			else {}

			return;
		}

		filter.update (motion, observationList);

		filter.getStates (statePtrList);
		tf::Transform curPose = summarizePF(statePtrList);
//		cout << curPose.getOrigin().x() << " " << curPose.getOrigin().y() << endl;
		camBr.sendTransform(tf::StampedTransform(curPose, imuMsg->header.stamp, odomParentFrameId, odomFrameId));

		prevTime = imuMsg->header.stamp.toSec();
		prevCount = imuMsg->wheel;

		// Debugging
		display (prevTime, observationList);
	}


protected:
	ParticleFilter<ImuState, tf::Transform, ImuMotionCtrl> filter;
	double prevTime;
	unsigned long int prevCount;
	bool positionFound;
	tf::Transform initialPose;
	tf::TransformBroadcaster camBr;
	vector<ImuState*> statePtrList;


	static constexpr double
		wyOffset = 0.0034,
		odomNoise1 = 0.00115,
		odomNoise2 = 0.00115,
		odomNoise3 = 0.00345,
		odomNoise4 = 0.00345,
		orbError = 0.5;


private:

	/*
	 * Create new state from previous state
	 * based on three parameters
	 */
	static ImuState calcOdometry (const ImuState &cstate, double v, double wy, double dt)
	{
		ImuState nstate;

		Eigen::Matrix<double, 3, 2> V;
		Eigen::Matrix<double, 2, 3> VT;
		Eigen::Matrix2d N;
		Eigen::Matrix3d Q, G, GT, SO, SN;

		double delD = v * dt,
			   delT = wy * dt;
		double roll, pitch, yaw;
		quaternionToEuler(cstate.orientation, roll, pitch, yaw);

		// renewal odometry
		nstate.position.x() = cstate.position.x() - delD * sin(yaw);
		nstate.position.y() = cstate.position.y() + delD * cos(yaw);
		yaw += 	delT;
//		eulerToQuaternion(roll, pitch, yaw, nstate)
//		nstate.position.x() = cstate.position.x() + delD * cos(cstate.orientation.z());
//		nstate.position.y() = cstate.position.y() + delD * sin(cstate.orientation.z());
//		nstate.orientation.z() = cstate.orientation.z() + delT;
//		nstate.orientation.w() = 1.0;
//		if (nstate.orientation.z() < -M_PI)
//			nstate.orientation.z() += 2.0 * M_PI;
//		else if (nstate.orientation.z() > M_PI)
//			nstate.orientation.z() -= 2.0 * M_PI;
//		nstate.twist_linear.x() = v * dt;
//		nstate.twist_angular.z() = wy * dt;

		return nstate;
	}


	static ImuState getAverage (const vector<ImuState> &states)
	{
		for (const ImuState &state: states) {

		}
	}


	static tf::Transform summarizePF (const vector<ImuState> &states)
	{
		tf::Transform avgPose;
		Eigen::Vector3d pos = Eigen::Vector3d::Zero();
		Eigen::Quaterniond ori = Eigen::Quaterniond::Identity();

		for (const auto &st: states) {
			pos.x() += st.position.x();
			pos.y() += st.position.y();
			pos.z() += st.position.z();
			// XXX: orientation not handled
		}

		pos.x() /= (double)states.size();
		pos.y() /= (double)states.size();
		pos.z() /= (double)states.size();

		convert(pos, ori, avgPose);
		return avgPose;
	}


	static tf::Transform summarizePF (const vector<ImuState*> &statePtr)
	{
		tf::Transform avgPose;
		Eigen::Vector3d pos = Eigen::Vector3d::Zero();
		Eigen::Quaterniond ori = Eigen::Quaterniond::Identity();

		for (const ImuState* st: statePtr) {
			pos.x() += st->position.x();
			pos.y() += st->position.y();
			pos.z() += st->position.z();
			// XXX: orientation not handled
		}

		pos.x() /= (double)statePtr.size();
		pos.y() /= (double)statePtr.size();
		pos.z() /= (double)statePtr.size();

		convert(pos, ori, avgPose);
		return avgPose;
	}



	static void convertVoltageToLinear ()
	{

	}


	void display (double timestamp, const vector<tf::Transform> &observations)
	{
		static FILE* gnuplot;
		FILE *particle_file, *realpos_file, *sens_file,*trajectory;
		int i;
		double mx, my, n;
		static int count;

		if (!gnuplot) {
			gnuplot = popen("/usr/bin/gnuplot", "w");
			fprintf (gnuplot, "set term x11\n");
		}

		sens_file = fopen("/tmp/sens_file", "w");
		if (observations.size()>0) {
			fprintf (sens_file, "%f %f 0 0 \n",
				observations[0].getOrigin().x(),
				observations[0].getOrigin().y());
		}
		if (observations.size()>1) {
			fprintf (sens_file, "0 0 %f %f \n",
				observations[1].getOrigin().x(),
				observations[1].getOrigin().y());
		}
		fclose(sens_file);

		particle_file = fopen("/tmp/particle_file", "w");
//		mx=my=n=0;
		for (i=0; i<filter.getNumberOfParticles(); i++) {
			const Particle<ImuState> &particle = filter.getParticle(i);
			fprintf (particle_file, "%f %f \n",
				particle.current.position.x(),
				particle.current.position.y());
//			mx += particle.current.position.x();
//			my += particle.current.position.y();
			n+=1;
		}
		fclose (particle_file);

//		trajectory = fopen ("trajectory", "a");
//		fprintf(particle_file, "%f %f %f\n",
//			timestamp,
//			mx/filter.getNumberOfParticles(),
//			my/filter.getNumberOfParticles());
//		fclose(trajectory);

		// Adjust from ground truth before running
		fprintf(gnuplot, "set xrange [%f:%f]\n",-600.0,1000.0);//mx/n-20,mx/n+20);
		fprintf(gnuplot, "set yrange [%f:%f]\n",-1500.0,1500.0);//my/n-15,my/n+15);

		fprintf(gnuplot, "plot '/tmp/particle_file' u 1:2 w d,");

		//  if(z->observed)
		fprintf(gnuplot, "'/tmp/sens_file' u 1:2, '/tmp/sens_file' u 3:4 \n");
		//  fprintf(gnuplot, "'realpos_file'  u 1:2\n");

		fflush(gnuplot);

	}

};



int main (int argc, char **argv)
{
	ros::init (argc, argv, "imu_fusion");
	ros::NodeHandle nh;

	ImuFusion prius;
	prius.addTfListener ("world", "orb1");
//	prius.addTfListener ("world", "orb2");

	ros::Subscriber priusImuSubscriber = nh.subscribe ("imu_wheel", 1, &ImuFusion::imuCallback, &prius);
	cout << "Subscribed" << endl;
	ros::spin();

	return 0;
}
