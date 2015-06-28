//---------------------------< /-/ AMSL /-/ >------------------------------
/**
 * file         :       roomba500sci.cpp
 *
 *
 * Environment  :       g++
 * Latest Update:       2011/05/24
 *
 * Designer(s)  :       t.saitoh (AMSL)
 * Author(s)    :       t.saitoh (AMSL)
 *
 * CopyRight    :       2011, Autonomous Mobile Systems Laboratory, Meiji Univ.
 *
 * Revision     :       2011/05/24
 *
 */
//-----------------------------------------------------------------------------

#include "roomba_500driver_meiji/roomba500sci.hpp"
#include "ros/ros.h"

#include <iostream>
#include <cmath>
using namespace std;

roombaSci::roombaSci(int baud, const char* dev)
  : enc_count_l_(0)
  , enc_count_r_(0)
  , d_enc_count_l_(0)
  , d_enc_count_r_(0)
  , d_pre_enc_l_(0)
  , d_pre_enc_r_(0)
{
  ser_ = new Serial(baud, dev, 80, 0);
  time_ = new Timer();
  time_ -> sleep(1);
  roomba_500driver_meiji::Roomba500State sensor;
  getSensors(sensor);
}

roombaSci::~roombaSci() {
  delete ser_;
  delete time_;
}

void roombaSci::wakeup(void) {
  ser_ -> setRts(0);
  time_ -> sleep(0.1);
  ser_ -> setRts(1);
  time_ -> sleep(COMMAND_WAIT);
}

void roombaSci::startup(void) {
  sendOPCODE(roombaSci::OC_START);
  sendOPCODE(roombaSci::OC_CONTROL);
}

void roombaSci::powerOff() {
  sendOPCODE(roombaSci::OC_POWER);
  time_ -> sleep(COMMAND_WAIT);
}

void roombaSci::clean() {
  sendOPCODE(roombaSci::OC_CLEAN);
  time_ -> sleep(COMMAND_WAIT);
}

void roombaSci::safe() {
  sendOPCODE(roombaSci::OC_SAFE);
  time_ -> sleep(COMMAND_WAIT);
}

void roombaSci::full() {
  sendOPCODE(roombaSci::OC_FULL);
  time_ -> sleep(COMMAND_WAIT);
}

void roombaSci::spot() {
  sendOPCODE(roombaSci::OC_SPOT);
  time_ -> sleep(COMMAND_WAIT);
}

void roombaSci::max() {
  sendOPCODE(roombaSci::OC_MAX);
  time_ -> sleep(COMMAND_WAIT);
}

void roombaSci::dock() {
  const unsigned char seq[] = {OC_BUTTONS, roombaSci::BB_DOCK};
  ser_ -> write(seq,2);
  time_ -> sleep(COMMAND_WAIT);
}

void roombaSci::driveMotors(roombaSci::MOTOR_STATE_BITS state) {
  const unsigned char seq[] = {OC_MOTORS, state};
  ser_ -> write(seq,2);
  time_ -> sleep(COMMAND_WAIT);
}

void roombaSci::seekDock() {
  const unsigned char seq[] = {OC_SEEK_DOCK};
  ser_ -> write(seq,1);
  time_ -> sleep(COMMAND_WAIT);
}

void roombaSci::drive(short velocity, short radius) {
  unsigned char vhi = (unsigned char)(velocity >> 8);
  unsigned char vlo = (unsigned char)(velocity & 0xff);
  unsigned char rhi = (unsigned char)(radius >> 8);
  unsigned char rlo = (unsigned char)(radius & 0xff);

  const unsigned char seq[] = {OC_DRIVE, vhi, vlo, rhi, rlo};
  ser_ -> write(seq, 5);
  time_ -> sleep(COMMAND_WAIT);
}

void roombaSci::driveDirect(float velocity, float yawrate) {
  short right = 1000 * (velocity + 0.5 * 0.235 * yawrate);
  short left = 1000 * (velocity - 0.5 * 0.235 * yawrate);

  unsigned char rhi = (unsigned char)(right >> 8);
  unsigned char rlo = (unsigned char)(right & 0xff);
  unsigned char lhi = (unsigned char)(left  >> 8);
  unsigned char llo = (unsigned char)(left  & 0xff);

  const unsigned char seq[] = {OC_DRIVE_DIRECT, rhi, rlo, lhi, llo};
  ser_ -> write(seq, 5);
  time_ -> sleep(COMMAND_WAIT);
}

void roombaSci::drivePWM(int right_pwm, int left_pwm) {
  short right = 255.0 / 100.0 * right_pwm;
  short left = 255.0 / 100.0 * left_pwm;

  unsigned char rhi = (unsigned char)(right >> 8);
  unsigned char rlo = (unsigned char)(right & 0xff);
  unsigned char lhi = (unsigned char)(left >> 8);
  unsigned char llo = (unsigned char)(left & 0xff);

  const unsigned char seq[] = {OC_DRIVE_PWM, rhi, rlo, lhi, llo};
  ser_ -> write(seq,5);
  time_ -> sleep(COMMAND_WAIT);
}

float roombaSci::velToPWM(float velocity) {
  if(velocity > 0)
    return 173.38 * velocity + 14.55; //ff para
  else if(velocity<0)
    return 206.00 * velocity - 13.25; //ff para
  else
    return 0;
}

void roombaSci::song(int song_number, int song_length) {
  const unsigned char mode_seq[] = {OC_SAFE};
  ser_ -> write(mode_seq, 1);
  time_ -> sleep(COMMAND_WAIT);

  const unsigned char command_seq[] = {OC_SONG, song_number, song_length, 60, 126};
  ser_ -> write(command_seq,2*song_length+3);
  time_ -> sleep(COMMAND_WAIT);
}

void roombaSci::playing(int song_number) {
  const unsigned char command_seq[] = {OC_PLAY, song_number};
  ser_ -> write(command_seq,2);
  time_ -> sleep(COMMAND_WAIT);
}

int roombaSci::sendOPCODE(roombaSci::OPCODE oc) {
  const unsigned char uc = (unsigned char)oc;
  int ret = ser_ -> write(&uc,1);
  time_ -> sleep(COMMAND_WAIT);
  return ret;
}

int roombaSci::receive(void) {
  return ser_ -> read(packet_, 80);
}

int roombaSci::receive(unsigned char* pack, int byte) {
  return ser_ -> read(pack, byte);
}

int roombaSci::getSensors(roomba_500driver_meiji::Roomba500State& sensor) {
  const unsigned char seq[] = {OC_SENSORS, ALL_PACKET};
  int ret = ser_ -> write(seq, 2);
  time_ -> sleep(COMMAND_WAIT);

  int nbyte;
  nbyte = receive();

  if(nbyte == 80)
    packetToStruct(sensor, packet_);

  time_ -> sleep(COMMAND_WAIT);
  return ret;
}

// TODO : Review
void roombaSci::packetToStruct(roomba_500driver_meiji::Roomba500State& ret,
			       const unsigned char* pack) {
  // Bumps and Wheel Drops
  ret.bumps_wheeldrops.bump_right = (bool)(pack[0] & BUMP_RIGHT);
  ret.bumps_wheeldrops.bump_left  = (bool)(pack[0] & BUMP_LEFT);
  ret.bumps_wheeldrops.wheeldrop_right = (bool)(pack[0] & WHEELDROP_RIGHT);
  ret.bumps_wheeldrops.wheeldrop_left  = (bool)(pack[0] & WHEELDROP_LEFT);
  // Wall
  ret.wall.wall = (bool)(pack[1]);
  // Cliff
  ret.cliff.left        = (bool)(pack[2] & 0x01);
  ret.cliff.front_left  = (bool)(pack[3] & 0x01);
  ret.cliff.front_right = (bool)(pack[4] & 0x01);
  ret.cliff.right       = (bool)(pack[5] & 0x01);
  // Vertial Wall
  ret.wall.vwall = (bool)(pack[6] & 0x01);
  // Wheel Overcurrents
  ret.wheel_overcurrents.left_wheel  = (bool)(pack[7] & LEFT_WHEEL);
  ret.wheel_overcurrents.right_wheel = (bool)(pack[7] & RIGHT_WHEEL);  // TODO : Add these message
  ret.wheel_overcurrents.main_brush  = (bool)(pack[7] & MAIN_BRUSH);
  ret.wheel_overcurrents.side_brush  = (bool)(pack[7] & SIDE_BRUSH);
  // Dirt Detect
  ret.dirt_detect = pack[8]; // TODO : Review variable type
  // Unused Byte
  // pack[9] is unused byte
  // IR Character Omni
  ret.ir_char.omni  = pack[10];
  ret.ir_char.left  = pack[11];
  ret.ir_char.right = pack[12];
  // Buttons
  ret.button.clean    = (bool)(pack[13] & BB_CLEAN);
  ret.button.spot     = (bool)(pack[13] & BB_SPOT);
  ret.button.dock     = (bool)(pack[13] & BB_DOCK);
  ret.button.minute   = (bool)(pack[13] & BB_MINUTE);
  ret.button.hour     = (bool)(pack[13] & BB_HOUR);
  ret.button.day      = (bool)(pack[13] & BB_DAY);
  ret.button.schedule = (bool)(pack[13] & BB_SCHEDULE);
  ret.button.clock    = (bool)(pack[13] & BB_CLOCK);
  // Traveled distance and angle
  ret.travel.distance = *((signed short*) (pack + 14)); // TODO : Add message
  ret.travel.angle = *((signed short*) (pack + 16));
  // Battery Charging state, Voltage, Current, Temperature, Charge, Capacity
  ret.battery.charging_state = pack[18];
  ret.battery.voltage = *((unsigned short*) (pack + 19));
  ret.battery.current = *((signed short*) (pack + 21));
  ret.battery.temperature = *((signed char*) (pack + 23));
  ret.battery.charge = *((unsigned short*) (pack + 24));
  ret.battery.capacity = *((unsigned short*) (pack + 26));
  // Wall Signal Strength
  ret.wall.wall_signal = *((unsigned short*) (pack + 28));
  // Cliff Signal Strength
  ret.cliff.left_signal        = *((unsigned short*) (pack + 30));
  ret.cliff.front_left_signal  = *((unsigned short*) (pack + 32));
  ret.cliff.front_right_signal = *((unsigned short*) (pack + 34));
  ret.cliff.right_signal       = *((unsigned short*) (pack + 36));
  // Unused Byte
  // pack[38, 39, 40] is unused byte
  // Charging sources availability
  ret.charging_source.home_base        = (bool)(pack[41] & HOME_BASE);
  ret.charging_source.internal_charger = (bool)(pack[41] & INTERNAL_CHARGER);
  // OI Mode
  ret.oi_mode = pack[42];
  // Song
  ret.song.number = pack[43]; // TODO : Check variable type
  ret.song.playing = pack[44];
  // #Stream packets
  ret.stream_packets = pack[45]; // TODO rename from oi_stream_num_packets
  // Requested Velocity and radius
  ret.request.velocity = *((signed short*) (pack + 46)); // TODO : Add message
  ret.request.radius   = *((signed short*) (pack + 48));
  ret.request.right_velocity = *((signed short*) (pack + 50));
  ret.request.left_velocity = *((signed short*) (pack + 52));
  // Encoder counts
  ret.encoder_counts.left  = *((unsigned short*) (pack + 54));
  ret.encoder_counts.right = *((unsigned short*) (pack + 56));
  // Light Bumper
  ret.light_bumper.left
    = (bool)(pack[58] & LT_BUMPER_LEFT);
  ret.light_bumper.front_left
    = (bool)(pack[58] & LT_BUMPER_FRONT_LEFT);
  ret.light_bumper.center_left
    = (bool)(pack[58] & LT_BUMPER_CENTER_LEFT);
  ret.light_bumper.center_right
    = (bool)(pack[58] & LT_BUMPER_CENTER_RIGHT);
  ret.light_bumper.front_right
    = (bool)(pack[58] & LT_BUMPER_FRONT_RIGHT);
  ret.light_bumper.right
    = (bool)(pack[58] & LT_BUMPER_RIGHT);
  // Light Bumper Signal Strength
  ret.light_bumper.left_signal
    = *((unsigned short*) (pack + 59));
  ret.light_bumper.front_left_signal
    = *((unsigned short*) (pack + 61));
  ret.light_bumper.center_left_signal
    = *((unsigned short*) (pack + 63));
  ret.light_bumper.center_right_signal
    = *((unsigned short*) (pack + 65));
  ret.light_bumper.front_right_signal
    = *((unsigned short*) (pack + 67));
  ret.light_bumper.right_signal
    = *((unsigned short*) (pack + 69));
  // Motor Current
  ret.motor_current.left_wheel  = *((signed short*) (pack + 71));
  ret.motor_current.right_wheel = *((signed short*) (pack + 73));
  ret.motor_current.main_brush  = *((signed short*) (pack + 75));
  ret.motor_current.side_brush  = *((signed short*) (pack + 77));
  // Stasis
  ret.stasis = (bool) pack[79];

  // Update internal encoder counts
  if(std::abs((int)ret.encoder_counts.right - (int)enc_count_r_) >= 60000) {
    if(ret.encoder_counts.right > enc_count_r_)
      d_enc_count_r_ = -65535 - enc_count_r_ + ret.encoder_counts.right;
    else
      d_enc_count_r_ = 65535 - enc_count_r_ + ret.encoder_counts.right;
  } else {
    d_enc_count_r_ = ret.encoder_counts.right - enc_count_r_;
  }

  if(std::abs((int)ret.encoder_counts.left - (int)enc_count_l_) >= 60000) {
    if(ret.encoder_counts.left > enc_count_l_)
      d_enc_count_l_ = -65535-enc_count_l_+ret.encoder_counts.left;
    else
      d_enc_count_l_ = 65535-enc_count_l_+ret.encoder_counts.left;
  } else {
    d_enc_count_l_ = ret.encoder_counts.left - enc_count_l_;
  }

  enc_count_r_ = ret.encoder_counts.right;
  enc_count_l_ = ret.encoder_counts.left;
}
