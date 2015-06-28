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
#include <roomba_500driver_meiji/Roomba500State.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include <boost/thread.hpp>
boost::mutex cntl_mutex_;

#include <iostream>
#include <math.h>
using namespace std;

roombaSci* roomba;
roomba_500driver_meiji::RoombaCtrl roombactrl;

void cntl_callback(const roomba_500driver_meiji::RoombaCtrlConstPtr& msg){
  roombactrl = *msg;
  boost::mutex::scoped_lock(cntl_mutex_);

  switch(msg->mode){
  case roomba_500driver_meiji::RoombaCtrl::SPOT:
    roomba->spot();
    break;

  case roomba_500driver_meiji::RoombaCtrl::SAFE:
    roomba->safe();
    break;

  case roomba_500driver_meiji::RoombaCtrl::CLEAN:
    roomba->clean();
    break;

  case roomba_500driver_meiji::RoombaCtrl::POWER:
    roomba->powerOff();
    break;

  case roomba_500driver_meiji::RoombaCtrl::WAKEUP:
    roomba->wakeup();
    roomba->startup();
    break;

  case roomba_500driver_meiji::RoombaCtrl::FULL:
    roomba->full();
    break;

  case roomba_500driver_meiji::RoombaCtrl::MAX:
    roomba->max();
    break;

  case roomba_500driver_meiji::RoombaCtrl::DOCK:
    roomba->dock();
    break;

  case roomba_500driver_meiji::RoombaCtrl::MOTORS:
    roomba->driveMotors((roombaSci::MOTOR_STATE_BITS)
			(roombaSci::MB_MAIN_BRUSH |
			 roombaSci::MB_SIDE_BRUSH |
			 roombaSci::MB_VACUUM));
    break;

  case roomba_500driver_meiji::RoombaCtrl::MOTORS_OFF:
    roomba->driveMotors((roombaSci::MOTOR_STATE_BITS)(0));
    break;

  case roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT:
    roomba->driveDirect(msg->cntl.linear.x, msg->cntl.angular.z);
    break;

  case roomba_500driver_meiji::RoombaCtrl::DRIVE_PWM:
    roomba->drivePWM(msg->r_pwm, msg->l_pwm);
    break;

  case roomba_500driver_meiji::RoombaCtrl::SONG:
    roomba->safe();
    roomba->song(1,1);
    roomba->playing(1);
    break;

  case roomba_500driver_meiji::RoombaCtrl::DRIVE:
  default:
    roomba->drive(msg->velocity, msg->radius);

  }
}

void printSensors(const roomba_500driver_meiji::Roomba500State& sens) {
  cout<<"\n\n-------------------"<<endl;
  cout << "Bumps:\n  "
       << (sens.bumps_wheeldrops.bump_right ? "Right, " : "")
       << (sens.bumps_wheeldrops.bump_left  ? "Left" : "") << endl;
  cout << "Wheeldrops:\n "
       << (sens.bumps_wheeldrops.wheeldrop_right ? "Right, " : "")
       << (sens.bumps_wheeldrops.wheeldrop_left  ? "Left" : "") << endl;
  cout << "Wall:\n  "
       << (sens.wall.wall ? "Wall, " : "")
       << (sens.wall.vwall ? "Virtual Wall" : "") << endl;
  cout <<"Cliff:\n  "
       << (sens.cliff.left ? "Left, " : "")
       << (sens.cliff.front_left ? "Front Left, " : "")
       << (sens.cliff.right ? "Right, " : "")
       << (sens.cliff.front_right ? "Front Right " : "") << endl;
  cout << "Wheel Overcurrent:\n  "
       << (sens.wheel_overcurrents.side_brush ? "Side Brush, " : "")
       << (sens.wheel_overcurrents.main_brush ? "Main Brush, " : "")
       << (sens.wheel_overcurrents.right_wheel ? "Right Wheel, " : "")
       << (sens.wheel_overcurrents.left_wheel ? "Left Wheel" : "") << endl;
  cout << "Dirt Detection:  " << sens.dirt_detect << endl;
  cout << "IR Character:"
       << "\n  Right: " << sens.ir_char.right
       << "\n  Left: "  << sens.ir_char.left
       << "\n  Omni: "  << sens.ir_char.omni << endl;
  cout << "Buttons:\n  "
       << (sens.button.clean    ? "Clean, " : "")
       << (sens.button.spot     ? "Spot, " : "")
       << (sens.button.dock     ? "Dock, " : "")
       << (sens.button.minute   ? "Minute, " : "")
       << (sens.button.hour     ? "Hour, " : "")
       << (sens.button.day      ? "Day, " : "")
       << (sens.button.clock    ? "Clock, " : "")
       << (sens.button.schedule ? "Schedule" : "") << endl;
  cout << "Travel:"
       << "\n  Distance: " << sens.travel.distance << "[mm?]"
       << "\n  Angle: " << sens.travel.angle << "[rad?]" << endl;
  cout << "Battery:"
       << "\n  Charging State: " << (uint)sens.battery.charging_state
       << "\n  Voltage: " << sens.battery.voltage
       << "\n  Current: " << sens.battery.current
       << "\n  Temperature: " << (int) sens.battery.temperature
       << "\n  Chage: " << sens.battery.charge
       << "\n  Capacity: " << sens.battery.capacity << endl;
  cout << "Charge Surce:\n  "
       << (sens.charging_source.home_base ? "Home Base, " : "")
       << (sens.charging_source.internal_charger ? "Internal Charger." : "") << endl;
  cout << "OI Mode : " << (int)sens.oi_mode << endl;
  cout << "Song:"
       << "\n  Numer:   " << sens.song.number
       << "\n  Playing: " << sens.song.playing << endl;
  cout << "Request:"
       << "\n  Velocity: " << sens.request.velocity
       << "\n  Radius: " << sens.request.radius
       << "\n  Right Velocity: " << sens.request.right_velocity
       << "\n  Left Velocity: " << sens.request.left_velocity << endl;
  cout << "Encoder Count:"
       << "\n  Right: " << sens.encoder_counts.right
       << "\n  Left : " << sens.encoder_counts.left << endl;
  cout << "Light Bumper: "
       << "\n  Left: " << sens.light_bumper.left_signal
       << ", " << (sens.light_bumper.left ? "Hit" : "Not Hit")
       << "\n  Right: " << sens.light_bumper.right_signal
       << ", " << (sens.light_bumper.right ? "Hit" : "Not Hit")
       << "\n  Front Left: " << sens.light_bumper.front_left_signal
       << ", " << (sens.light_bumper.front_left ? "Hit" : "Not Hit")
       << "\n  Front Right: " << sens.light_bumper.front_right_signal
       << ", " << (sens.light_bumper.front_right ? "Hit" : "Not Hit")
       << "\n  Center Left: " << sens.light_bumper.center_left_signal
       << ", " << (sens.light_bumper.center_left ? "Hit" : "Not Hit")
       << "\n  Center Right: " << sens.light_bumper.center_right_signal
       << ", " << (sens.light_bumper.center_right ? "Hit" : "Not Hit") << endl;
  cout << "Motor Current:"
       << "\n  Left Wheel: " << sens.motor_current.left_wheel
       << "\n  Right Wheel: " << sens.motor_current.right_wheel
       << "\n  Main Brush: " << sens.motor_current.main_brush
       << "\n  Side Brush: " << sens.motor_current.side_brush << endl;
  cout << "Stasis: " << (sens.stasis ? "Forward" : "Not Forward") << endl;
}

double piToPI(double rad) {
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
  x.theta=piToPI(x.theta);
  x.x=pre_x.x+dist*cos(x.theta);
  x.y=pre_x.y+dist*sin(x.theta);
}

int main(int argc, char** argv) {
  roomba = new roombaSci(B115200,"/dev/ttyUSB0");
  roomba->wakeup();
  roomba->startup();

  ros::init(argc, argv, "roomba_driver");
  ros::NodeHandle n;
  ros::Subscriber cntl_sub = n.subscribe("/roomba/control", 100, cntl_callback);
  ros::Publisher pub_state = n.advertise<roomba_500driver_meiji::Roomba500State>("/roomba/states", 100);

  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher pub_odo= n.advertise<nav_msgs::Odometry >("/roomba/odometry", 100);
  ros::Rate loop_rate(10); // should not set this rate more than 20Hz !

  geometry_msgs::Pose2D pose;
  pose.x=0;	pose.y=0;	pose.theta=0;

  int pre_enc_r=0, pre_enc_l=0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  while (ros::ok()) {
    current_time = ros::Time::now();

    roomba_500driver_meiji::Roomba500State sens;
    sens.header.stamp=ros::Time::now();
    {
      boost::mutex::scoped_lock(cntl_mutex_);
      roomba->getSensors(sens);
    }
    printSensors(sens);

    int enc_r = roomba->dEncoderRight();
    int enc_l = roomba->dEncoderLeft();
    if(abs(enc_r) == 200)
      enc_r = pre_enc_r;
    if(abs(enc_l) == 200)
      enc_l = pre_enc_l;

    geometry_msgs::Pose2D pre=pose;
    float distance = (float)((float)enc_r + (float)enc_l) / 2270.0 * 0.5;
    float angle    = (float)((float)enc_r - (float)enc_l) / 2270.0 / 0.235;
    //sens.travel.distance=(short)(1000 * distance);
    //sens.travel.angle=(short)(angle * 180.0 / M_PI);

    pub_state.publish(sens);
    calcOdometry(pose, pre, distance, angle);
    pre_enc_r=roomba->dEncoderRight();
    pre_enc_l=roomba->dEncoderLeft();

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

    //set the position
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = roombactrl.cntl.linear.x;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = roombactrl.cntl.angular.z;


    pub_odo.publish(odom);

    last_time = current_time;

    ros::spinOnce();
    loop_rate.sleep();
    /*
    ROS_INFO("dt:%f\t444444444l: %5d\tr:%5d\tdl %4d\tdr %4d\tx:%f\ty:%f\ttheta:%f",
	     last_time.toSec()-current_time.toSec(),
	     sens.encoder_counts.left, sens.encoder_counts.right,
	     roomba->dEncoderLeft(),
	     roomba->dEncoderRight(),
	     pose.x, pose.y, pose.theta/M_PI*180.0);
    */
  }
  roomba->powerOff();
  roomba->time_->sleep(1);
  delete roomba;
  return 0;
}
