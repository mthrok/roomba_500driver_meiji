#include "ros/ros.h"

#include <create2_msgs/RoombaCtrl.h>
#include <create2_msgs/RoombaSensors.h>
#include "create2_driver/create2sci.hpp"

#include <iostream>
#include <math.h>

using namespace std;

typedef create2_msgs::RoombaSensors RoombaSensors;
typedef create2_msgs::RoombaCtrl    RoombaCtrl;

void printSensorState(RoombaSensors sens) {
  cout << "\n\n-------------------" << endl
       << "Bumps:\n  "
       << (sens.bumps_wheeldrops.bump_right ? "Right, " : "       ")
       << (sens.bumps_wheeldrops.bump_left  ? "Left" : "") << endl
       << "Wheeldrops:\n "
       << (sens.bumps_wheeldrops.wheeldrop_right ? "Right, " : "       ")
       << (sens.bumps_wheeldrops.wheeldrop_left  ? "Left" : "") << endl
       << "Wall:\n  "
       << (sens.wall.wall ? "Wall, " : "      ")
       << (sens.wall.vwall ? "Virtual Wall" : "") << endl
       << "Cliff:\n  "
       << (sens.cliff.left ? "Left, " : "")
       << (sens.cliff.front_left ? "Front Left, " : "")
       << (sens.cliff.right ? "Right, " : "")
       << (sens.cliff.front_right ? "Front Right " : "") << endl
       << "Wheel Overcurrent:\n  "
       << (sens.wheel_overcurrents.side_brush ? "Side Brush, " : "")
       << (sens.wheel_overcurrents.main_brush ? "Main Brush, " : "")
       << (sens.wheel_overcurrents.right_wheel ? "Right Wheel, " : "")
       << (sens.wheel_overcurrents.left_wheel ? "Left Wheel" : "") << endl
       << "Dirt Detection:  " << (int)sens.dirt_detect << endl
       << "IR Character:"
       << "\n  Right: " << (int)sens.ir_opcodes.right
       << "\n  Left: "  << (int)sens.ir_opcodes.left
       << "\n  Omni: "  << (int)sens.ir_opcodes.omni << endl
       << "Buttons:\n  "
       << (sens.button.clean    ? "Clean, " : "")
       << (sens.button.spot     ? "Spot, " : "")
       << (sens.button.dock     ? "Dock, " : "")
       << (sens.button.minute   ? "Minute, " : "")
       << (sens.button.hour     ? "Hour, " : "")
       << (sens.button.day      ? "Day, " : "")
       << (sens.button.clock    ? "Clock, " : "")
       << (sens.button.schedule ? "Schedule" : "") << endl
       << "Traveled (From the last acquisition.):"
       << "\n  Distance: " << sens.travel.distance << " [mm]"
       << "\n  Angle: " << sens.travel.angle << " [deg]" << endl
       << "Battery:"
       << "\n  Charging State: " << (uint)sens.battery.charging_state
       << "\n  Voltage: " << sens.battery.voltage
       << "\n  Current: " << sens.battery.current
       << "\n  Temperature: " << (int) sens.battery.temperature
       << "\n  Ramains: "
       << sens.battery.charge << " / " << sens.battery.capacity
       << " (" << 100.0f * sens.battery.charge / sens.battery.capacity << "[%])"
       << endl
       << "Charge Source:\n  "
       << (sens.charging_source.home_base ? "Home Base, " : "")
       << (sens.charging_source.internal_charger ? "Internal Charger." : "")
       << endl
       << "OI Mode : " << (int)sens.oi_mode << endl
       << "Song:"
       << "\n  Numer:   " << (int)sens.song.number
       << "\n  Playing: " << (int)sens.song.playing << endl
       << "Request:"
       << "\n  Velocity: " << sens.request.velocity
       << "\n  Radius: " << sens.request.radius
       << "\n  Right Velocity: " << sens.request.right_velocity
       << "\n  Left Velocity: " << sens.request.left_velocity << endl
       << "Encoder Count:"
       << "\n  L / R: " << sens.encoder_counts.right
       << " / " << sens.encoder_counts.left << endl
       << "Light Bumper: "
       << "\n  Left:         "
       << (sens.light_bumper.left ? "    Hit" : "Not Hit")
       << ", (" << sens.light_bumper.left_signal << ")"
       << "\n  Right:        "
       << (sens.light_bumper.right ? "    Hit" : "Not Hit")
       << ", (" << sens.light_bumper.right_signal << ")"
       << "\n  Front Left:   "
       << (sens.light_bumper.front_left ? "    Hit" : "Not Hit")
       << ", (" << sens.light_bumper.front_left_signal << ")"
       << "\n  Front Right:  "
       << (sens.light_bumper.front_right ? "    Hit" : "Not Hit")
       << ", (" << sens.light_bumper.front_right_signal << ")"
       << "\n  Center Left:  "
       << (sens.light_bumper.center_left ? "    Hit" : "Not Hit")
       << ", (" << sens.light_bumper.center_left_signal << ")"
       << "\n  Center Right: "
       << (sens.light_bumper.center_right ? "    Hit" : "Not Hit")
       << ", (" << sens.light_bumper.center_right_signal << ")" << endl
       << "Motor Current:"
       << "\n  Left Wheel: " << sens.motor_current.left_wheel
       << "\n  Right Wheel: " << sens.motor_current.right_wheel
       << "\n  Main Brush: " << sens.motor_current.main_brush
       << "\n  Side Brush: " << sens.motor_current.side_brush << endl
       << "Stasis: " << (sens.stasis ? "Forward" : "Not Forward") << endl;
}

int main(int argc, char** argv) {
  int rate = 30;

  ros::init(argc, argv, "roomba_driver");

  create2::Roomba roomba;
  roomba.init(B115200, "/dev/ttyUSB0");
  roomba.wakeup();
  roomba.startup();

  ros::NodeHandle hNode;
  ros::Rate loop_rate(rate);
  ros::Subscriber sub_ctrl = hNode.subscribe("/roomba/control", 100, &create2::Roomba::sendCtrl, &roomba);
  ros::Publisher pub_state = hNode.advertise<RoombaSensors>("/roomba/sensors", 100);

  while (ros::ok()) {
    RoombaSensors sens = roomba.getSensorState();
    sens.header.stamp = ros::Time::now();

    ros::spinOnce();
    loop_rate.sleep();
  }

  roomba.powerOff();
  return 0;
}
