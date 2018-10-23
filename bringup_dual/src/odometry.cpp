#include <math.h>

#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listner.h>

#include "bringup_dual/commendMsg.h"
#include "bringup_dual/motorsMsg.h"
#include "bringup_dual/realVel.h"



#define PI 3.14159265358979323846

double realVel[4];



void measureCallback(const bringup_dual::realVel::ConstPtr& msg)
{

	realVel[4] = {msg->realVel[0], msg->realVel[1], msg->realVel[2], msg->realVel[3]}

}


tf::Transform getTransformForMotion(
	double linear_vel_x,
	double linear_vel_y,
	double angular_vel_z,
	double timeSeconds
) const
{

    linear_vel_x =
		wheel_radius/4.0*(wheel_speed_lf+wheel_speed_rf+wheel_speed_lb+wheel_speed_rb);

    linear_vel_y =
		wheel_radius/4.0*(-wheel_speed_lf+wheel_speed_rf+wheel_speed_lb-wheel_speed_rb);

    angular_vel_z =
		wheel_radius/(4.0*l)*(-wheel_speed_lf+wheel_speed_rf-wheel_speed_lb+wheel_speed_rb);

	tf::Transform tmp;
	tmp.setIdentity();

	if (std::abs(angular_vel_z) < 0.0001){
		tmp.setOrigin(
			tf::Vector3(
				static_cast<double>(u_x*timeSeconds),
				static_cast<double>(linear_vel_x*timeSeconds),
				0.0
			)
		);
	}else{
		double angleChange = angular_vel_z*timeSeconds
		tmp.setOrigin(
			tf::Vector3(
				static_cast<double>(linear_vel_x*timeSeconds),
				static_cast<double>(linear_vel_x*timeSeconds),
				0.0
			)
		);
		tmp.setRotation(tf::createQuaternionFromYaw(angleChange));
	}
	return tmp;

}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "odom_publisher");
	ros::NodeHandle nh;


	nav_msgs::Odometry odom

	ros::Subscriber measure_sub = nh.subscribe("/measure", 100, measureCallback);

	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);

	boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster;


	ros::Rate loop_rate(50);
	

	//Gear ratio
	int gear_ratio = 76;
	//radps_to_rpm : rad/sec --> rpm
	double radps_to_rpm = 60.0/2.0/PI;
	//rpm_to_radps : rpm --> rad/sec
	double rpm_to_radps = 2.0*PI/60;

	//Wheel specification in meter
	double wheel_diameter = 0.152;
	double wheel_radius = wheel_diameter/2.0;
	double wheel_separation_a = 0.2355;
	double wheel_separation_b = 0.281;
	double l = wheel_separation_a+wheel_separation_b;

	//Motor speed in rad/sec - initialization
	double wheel_speed_lf = 0;
	double wheel_speed_rf = 0;
	double wheel_speed_lb = 0;
	double wheel_speed_rb = 0;

	double linear_vel_x = 0;
	double linear_vel_y = 0;
	double angular_vel_z = 0;


	while(ros::ok())
	{
		ros::time currentTime = ros::Time::now();
		std::string odom_frame = tf::resolve(tf_prefix, odometry_frame);
		std::string base_footprint_frame = tf::resolve(tf_prefix, robot_vase_frame);

		linear_vel_x =
			wheel_radius/4.0*(wheel_speed_lf+wheel_speed_rf+wheel_speed_lb+wheel_speed_rb);

		linear_vel_y =
			wheel_radius/4.0*(-wheel_speed_lf+wheel_speed_rf+wheel_speed_lb-wheel_speed_rb);

		angular_vel_z =
			wheel_radius/(4.0*l)*(-wheel_speed_lf+wheel_speed_rf-wheel_speed_lb+wheel_speed_rb);
		
		odom_transform =
			odom_transform*getTransformForMotion(
				linear_vel_x, linear_vel_y, angular_vel_z, step_time);

		tf::poseTFToMsg(odom_transform, odom.pose.pose);

		odom.twist.twist.angular.z = angular_vel_z;
		odom.twist.twist.linear.x  = linear_vel_x;
		odom.twist.twist.linear.y  = linear_vel_y;

		odom.header.stamp = current_time;
		odom.header.frame_id = odom_frame;
		odom.child_frame_id = base_footprint_frame;

		if (transform_broadcaster.get()){
			transform_broadcaster_->sendTransform(
				tf::StampedTransform(
					odom_transform,
					current_time,
					odom_frame,
					base_footprint_frame
				)
			);
		}
		
		odom.pose.covariance[0] = 0.001;
		odom.pose.covariance[7] = 0.001;
		odom.pose.covariance[14] = 1000000000000.0;
		odom.pose.covariance[21] = 1000000000000.0;
		odom.pose.covariance[28] = 1000000000000.0;
		
		if (std::abs(angular_vel_z) < 0.0001) {
			odom.pose.covariance[35] = 0.01;
		}else{
			odom.pose.covariance[35] = 100.0;
		}

		odom.twist.covariance[0] = 0.001;
		odom.twist.covariance[7] = 0.001;
		odom.twist.covariance[14] = 0.001;
		odom.twist.covariance[21] = 1000000000000.0;
		odom.twist.covariance[28] = 1000000000000.0;
		
		if (std::abs(angular_vel_z) < 0.0001) {
			odom.twist.covariance[35] = 0.01;
		}else{
			odom.twist.covariance[35] = 100.0;
		}

		odom_pub.publish(odom)
	}

	return 0;

}
