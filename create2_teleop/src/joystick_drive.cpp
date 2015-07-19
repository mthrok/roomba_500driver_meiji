#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include <sensor_msgs/Joy.h>
#include <create2_msgs/RoombaCtrl.h>

using namespace std;
boost::mutex cntl_mutex_;
sensor_msgs::Joy joy_in;
ros::Publisher pub_state;

void joy_callback(const sensor_msgs::JoyConstPtr& msg){
  boost::mutex::scoped_lock(cntl_mutex_);
  create2_msgs::RoombaCtrl ctrl;

  ctrl.mode=create2_msgs::RoombaCtrl::DRIVE;

  if(msg->buttons[0]){
    ctrl.mode=create2_msgs::RoombaCtrl::SAFE;
    cout<<"SAFE mode"<<endl;
  }

  if(msg->buttons[1]){
    ctrl.mode=create2_msgs::RoombaCtrl::SPOT;
    cout<<"SPOT mode"<<endl;
  }

  if(msg->buttons[2]){
    ctrl.mode=create2_msgs::RoombaCtrl::CLEAN;
    cout<<"CLEAN mode"<<endl;
  }

  if(msg->buttons[3]){
    //ctrl.mode=create2_msgs::RoombaCtrl::MAX;
    //cout<<"MAX mode"<<endl;
    ctrl.mode=create2_msgs::RoombaCtrl::DOCK;
    cout<<"DOCK mode"<<endl;
  }

  if(msg->buttons[4]){
    ctrl.mode=create2_msgs::RoombaCtrl::MOTORS;
    cout<<"MOTORS mode"<<endl;
  }
  if(msg->buttons[6]){
    ctrl.mode=create2_msgs::RoombaCtrl::MOTORS_OFF;
    cout<<"MOTORS OFF mode"<<endl;
	}
  if(msg->buttons[5]){
    ctrl.mode=create2_msgs::RoombaCtrl::FORCE_SEEK_DOCK;
    cout<<"FORCE_SEEK_DOCK mode"<<endl;
  }
  if(msg->buttons[7]){
    ctrl.mode=create2_msgs::RoombaCtrl::FULL;
    cout<<"FULL mode"<<endl;
  }

  if(msg->buttons[9]){	//for red controller
    //if(msg->buttons[10]){
    ctrl.mode=create2_msgs::RoombaCtrl::WAKEUP;
    cout<<"WAKEUP"<<endl;
  }

  if(msg->buttons[8]){	//for red
    //if(msg->buttons[11]){
    ctrl.mode=create2_msgs::RoombaCtrl::POWER;
    cout<<"POWER OFF"<<endl;
  }
  /*
    ctrl.velocity=1.0*roomba_msgs::RoombaCtrl::DEFAULT_VELOCITY*msg->axes[1]+
    roomba_msgs::RoombaCtrl::DEFAULT_VELOCITY*msg->axes[5];

    ctrl.radius=((float)ctrl.velocity/msg->axes[2])+0*roomba_msgs::RoombaCtrl::STRAIGHT_RADIUS;

    if(fabs(ctrl.radius)>roomba_msgs::RoombaCtrl::STRAIGHT_RADIUS){
    ctrl.radius=roomba_msgs::RoombaCtrl::STRAIGHT_RADIUS;
    }

    if(msg->axes[4]){ //
    ctrl.velocity=roomba_msgs::RoombaCtrl::DEFAULT_VELOCITY;
    ctrl.radius=msg->axes[4];
    }
  */
  if(msg->axes[1] || msg->axes[2]){ //
    ctrl.cntl.linear.x=0.002*create2_msgs::RoombaCtrl::DEFAULT_VELOCITY*msg->axes[1];
    ctrl.cntl.angular.z=(float)msg->axes[2];

    ctrl.mode=create2_msgs::RoombaCtrl::DRIVE_DIRECT;
    //cout<<ctrl.velocity<<"   "<<ctrl.radius<<endl;
    cout<<ctrl.cntl.linear.x<<"   "<<ctrl.cntl.angular.z<<endl;
  }

  pub_state.publish(ctrl);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick_drive");
    ros::NodeHandle n;

    pub_state=n.advertise<create2_msgs::RoombaCtrl>("/roomba/control", 100);
    ros::Subscriber cntl_sub = n.subscribe("/joy", 100, joy_callback);

    ros::spin();
    return 0;
}
