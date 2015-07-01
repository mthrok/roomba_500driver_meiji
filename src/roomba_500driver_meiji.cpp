//---------------------------< /-/ AMSL /-/ >------------------------------
/**
 * file         :       main500.cpp
 *
 *
 * Environment  :       g++
 * Latest Update:       2011/05/24
 *
 * Designer(s)  :       t.saitoh (AMSL)
 * Author(s)    :       m.mitsuhashi (AMSL)
 *
 * CopyRight    :       2011, Autonomous Mobile Systems Laboratory, Meiji Univ.
 *
 * Revision     :       2011/05/24
 *
 */
//-----------------------------------------------------------------------------

#include "ros/ros.h"

#include "roomba_500driver_meiji/roomba500sci.hpp"
#include <roomba_500driver_meiji/RoombaState.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <math.h>

using namespace std;

typedef roomba_500driver_meiji::RoombaState RoombaState;
typedef roomba_500driver_meiji::RoombaCtrl RoombaCtrl;

double normalizeRad(double rad) {
  double ret=rad;
  if(rad>M_PI)
    ret=rad-2.0*M_PI;
  if(rad<-M_PI)
    ret=rad+2.0*M_PI;
  return ret;
}

void calcOdometry (geometry_msgs::Pose2D& x,
		   const geometry_msgs::Pose2D& pre_x,
		   float dist,
		   float angle) {
  x.theta=pre_x.theta+angle;
  x.theta=normalizeRad(x.theta);
  x.x=pre_x.x+dist*cos(x.theta);
  x.y=pre_x.y+dist*sin(x.theta);
}

int main(int argc, char** argv) {
  roombaC2::Roomba roomba;
  roomba.init(B115200,"/dev/ttyUSB0");
  roomba.wakeup();
  roomba.startup();

  ros::init(argc, argv, "roomba_driver");
  ros::NodeHandle n;
  ros::Subscriber cntl_sub = n.subscribe("/roomba/control", 100, &roombaC2::Roomba::sendCtrl, &roomba);
  ros::Publisher pub_state = n.advertise<RoombaState>("/roomba/states", 100);

  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher pub_odo= n.advertise<nav_msgs::Odometry >("/roomba/odometry", 100);
  ros::Rate loop_rate(10); // should not set this rate more than 20Hz !

  geometry_msgs::Pose2D pose;
  pose.x=0;	pose.y=0;	pose.theta=0;

  int pre_enc_r=0, pre_enc_l=0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  while (ros::ok()) {
    current_time = ros::Time::now();
    roomba.updateSensorState();
    roomba.setTimestamp(current_time);
    roomba.printSensorState();

    int enc_r = roomba.dEncoderRight();
    int enc_l = roomba.dEncoderLeft();
    if(abs(enc_r) == 200)
      enc_r = pre_enc_r;
    if(abs(enc_l) == 200)
      enc_l = pre_enc_l;

    geometry_msgs::Pose2D pre = pose;
    float distance = ((float)enc_r + enc_l) / 2270.0 * 0.5;
    float angle    = ((float)enc_r - enc_l) / 2270.0 / 0.235;
    roomba.setTravelDistance(distance);
    roomba.setTravelAngle(angle);

    pub_state.publish(roomba.getSensorState());

    calcOdometry(pose, pre, distance, angle);
    pre_enc_r=roomba.dEncoderRight();
    pre_enc_l=roomba.dEncoderLeft();

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.theta);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = pose.x;
    odom_trans.transform.translation.y = pose.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = roomba.getCtrlLinearX();
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = roomba.getCtrlAngleZ();
    pub_odo.publish(odom);


    ros::spinOnce();
    loop_rate.sleep();
  }
  roomba.powerOff();
  return 0;
}
