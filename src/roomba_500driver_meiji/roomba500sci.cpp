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

#include <unistd.h>
#include <cmath>

typedef unsigned short uint16;
typedef unsigned char uint8;
typedef signed short int16;
typedef signed char int8;

using namespace std;
using namespace roombaC2;

void sleep_for_sec (float sec){
  long usec=(long)(sec*1000000);
  usleep(usec);
}

Roomba::Roomba()
  : enc_count_l_(0)
  , enc_count_r_(0)
  , d_enc_count_l_(0)
  , d_enc_count_r_(0)
  , d_pre_enc_l_(0)
  , d_pre_enc_r_(0)
{}

Roomba::~Roomba() {
  delete ser_;
}

void Roomba::init(int baud, const char* dev) {
  ser_ = new Serial(baud, dev, 80, 0);
  sleep(1);

  roomba_500driver_meiji::Roomba500State sensor;
  getSensors(sensor);
};

void Roomba::wakeup(void) {
  ser_ -> setRts(0);
  sleep_for_sec(0.1);
  ser_ -> setRts(1);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::startup(void) {
  sendOPCODE(OC_START);
  sendOPCODE(OC_CONTROL);
}

void Roomba::powerOff() {
  sendOPCODE(OC_POWER);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::clean() {
  sendOPCODE(OC_CLEAN);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::safe() {
  sendOPCODE(OC_SAFE);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::full() {
  sendOPCODE(OC_FULL);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::spot() {
  sendOPCODE(OC_SPOT);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::max() {
  sendOPCODE(OC_MAX);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::dock() {
  const uint8 seq[] = {OC_BUTTONS, BB_DOCK};
  ser_ -> write(seq,2);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::driveMotors(MOTOR_STATE_BITS state) {
  const uint8 seq[] = {OC_MOTORS, state};
  ser_ -> write(seq,2);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::seekDock() {
  const uint8 seq[] = {OC_SEEK_DOCK};
  ser_ -> write(seq,1);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::drive(short velocity, short radius) {
  uint8 vhi = (uint8)(velocity >> 8);
  uint8 vlo = (uint8)(velocity & 0xff);
  uint8 rhi = (uint8)(radius >> 8);
  uint8 rlo = (uint8)(radius & 0xff);

  const uint8 seq[] = {OC_DRIVE, vhi, vlo, rhi, rlo};
  ser_ -> write(seq, 5);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::driveDirect(float velocity, float yawrate) {
  short right = 1000 * (velocity + 0.5 * 0.235 * yawrate);
  short left = 1000 * (velocity - 0.5 * 0.235 * yawrate);

  uint8 rhi = (uint8)(right >> 8);
  uint8 rlo = (uint8)(right & 0xff);
  uint8 lhi = (uint8)(left  >> 8);
  uint8 llo = (uint8)(left  & 0xff);

  const uint8 seq[] = {OC_DRIVE_DIRECT, rhi, rlo, lhi, llo};
  ser_ -> write(seq, 5);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::drivePWM(int right_pwm, int left_pwm) {
  short right = 255.0 / 100.0 * right_pwm;
  short left = 255.0 / 100.0 * left_pwm;

  uint8 rhi = (uint8)(right >> 8);
  uint8 rlo = (uint8)(right & 0xff);
  uint8 lhi = (uint8)(left >> 8);
  uint8 llo = (uint8)(left & 0xff);

  const uint8 seq[] = {OC_DRIVE_PWM, rhi, rlo, lhi, llo};
  ser_ -> write(seq,5);
  sleep_for_sec(COMMAND_WAIT);
}

float Roomba::velToPWM(float velocity) {
  if(velocity > 0)
    return 173.38 * velocity + 14.55; //ff para
  else if(velocity<0)
    return 206.00 * velocity - 13.25; //ff para
  else
    return 0;
}

void Roomba::song(int song_number, int song_length) {
  const uint8 mode_seq[] = {OC_SAFE};
  ser_ -> write(mode_seq, 1);
  sleep_for_sec(COMMAND_WAIT);

  const uint8 command_seq[] = {OC_SONG, song_number, song_length, 60, 126};
  ser_ -> write(command_seq,2*song_length+3);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::playing(int song_number) {
  const uint8 command_seq[] = {OC_PLAY, song_number};
  ser_ -> write(command_seq,2);
  sleep_for_sec(COMMAND_WAIT);
}

int Roomba::sendOPCODE(OPCODE oc) {
  const uint8 uc = (uint8)oc;
  int ret = ser_ -> write(&uc,1);
  sleep_for_sec(COMMAND_WAIT);
  return ret;
}

int Roomba::receive(void) {
  return ser_ -> read(packet_, 80);
}

int Roomba::receive(uint8* pack, int byte) {
  return ser_ -> read(pack, byte);
}

int Roomba::getSensors(roomba_500driver_meiji::Roomba500State& sensor) {
  const uint8 seq[] = {OC_SENSORS, ALL_PACKET};
  int ret = ser_ -> write(seq, 2);
  sleep_for_sec(COMMAND_WAIT);

  int nbyte;
  nbyte = receive();

  if(nbyte == 80)
    packetToStruct(sensor, packet_);

  sleep_for_sec(COMMAND_WAIT);
  return ret;
}

int Roomba::dEncoderRight(int max_delta) {
  d_enc_count_r_ = std::max(-max_delta, d_enc_count_r_);
  d_enc_count_r_ = std::min(max_delta, d_enc_count_r_);
  return d_enc_count_r_;
};

int Roomba::dEncoderLeft(int max_delta) {
  d_enc_count_l_ = std::max(-max_delta, d_enc_count_l_);
  d_enc_count_l_ = std::min(max_delta, d_enc_count_l_);
  return d_enc_count_l_;
}

void Roomba::packetToStruct(roomba_500driver_meiji::Roomba500State& ret,
			       const uint8* pack) {
  // Bumps and Wheel Drops
  ret.bumps_wheeldrops.bump_right      = (pack[0] & BUMP_RIGHT);
  ret.bumps_wheeldrops.bump_left       = (pack[0] & BUMP_LEFT);
  ret.bumps_wheeldrops.wheeldrop_right = (pack[0] & WHEELDROP_RIGHT);
  ret.bumps_wheeldrops.wheeldrop_left  = (pack[0] & WHEELDROP_LEFT);
  // Wall
  ret.wall.wall                        = (pack[1] & 0x01);
  // Cliff
  ret.cliff.left                       = (pack[2] & 0x01);
  ret.cliff.front_left                 = (pack[3] & 0x01);
  ret.cliff.front_right                = (pack[4] & 0x01);
  ret.cliff.right                      = (pack[5] & 0x01);
  // Vertial Wall
  ret.wall.vwall                       = (pack[6] & 0x01);
  // Wheel Overcurrents
  ret.wheel_overcurrents.left_wheel    = (pack[7] & LEFT_WHEEL);
  ret.wheel_overcurrents.right_wheel   = (pack[7] & RIGHT_WHEEL);
  ret.wheel_overcurrents.main_brush    = (pack[7] & MAIN_BRUSH);
  ret.wheel_overcurrents.side_brush    = (pack[7] & SIDE_BRUSH);
  // Dirt Detect
  ret.dirt_detect                      = pack[8];
  // Unused Byte
  // pack[9] is unused byte
  // IR Opcodes
  ret.ir_opcodes.omni                  = pack[10];
  ret.ir_opcodes.left                  = pack[69]; // <- Attention
  ret.ir_opcodes.right                 = pack[70]; // <- It's confusing.
  // Buttons
  ret.button.clean                     = (pack[11] & BB_CLEAN);
  ret.button.spot                      = (pack[11] & BB_SPOT);
  ret.button.dock                      = (pack[11] & BB_DOCK);
  ret.button.minute                    = (pack[11] & BB_MINUTE);
  ret.button.hour                      = (pack[11] & BB_HOUR);
  ret.button.day                       = (pack[11] & BB_DAY);
  ret.button.schedule                  = (pack[11] & BB_SCHEDULE);
  ret.button.clock                     = (pack[11] & BB_CLOCK);
  // Traveled distance and angle
  ret.travel.distance                  = *((int16*) (pack + 12));
  ret.travel.angle                     = *((int16*) (pack + 14));
  // Battery Charging state, Voltage, Current, Temperature, Charge, Capacity
  ret.battery.charging_state           = pack[16];
  ret.battery.voltage                  = *((uint16*) (pack + 17));
  ret.battery.current                  = *((int16*)  (pack + 19));
  ret.battery.temperature              = *((int8*)   (pack + 21));
  ret.battery.charge                   = *((uint16*) (pack + 22));
  ret.battery.capacity                 = *((uint16*) (pack + 24));
  // Wall Signal Strength
  ret.wall.wall_signal                 = *((uint16*) (pack + 26));
  // Cliff Signal Strength
  ret.cliff.left_signal                = *((uint16*) (pack + 28));
  ret.cliff.front_left_signal          = *((uint16*) (pack + 30));
  ret.cliff.front_right_signal         = *((uint16*) (pack + 32));
  ret.cliff.right_signal               = *((uint16*) (pack + 34));
  // Unused Byte
  // pack[36, 37, 38] is unused byte
  // Charging sources availability
  ret.charging_source.home_base        = (pack[39] & HOME_BASE);
  ret.charging_source.internal_charger = (pack[39] & INTERNAL_CHARGER);
  // OI Mode
  ret.oi_mode = pack[40];
  // Song
  ret.song.number  = pack[41];
  ret.song.playing = pack[42];
  // #Stream packets
  ret.stream_packets = pack[43];
  // Requested Velocity and radius
  ret.request.velocity                 = *((int16*) (pack + 44));
  ret.request.radius                   = *((int16*) (pack + 46));
  ret.request.right_velocity           = *((int16*) (pack + 48));
  ret.request.left_velocity            = *((int16*) (pack + 50));
  // Encoder counts
  ret.encoder_counts.left              = *((uint16*) (pack + 52));
  ret.encoder_counts.right             = *((uint16*) (pack + 54));
  // Light Bumper
  ret.light_bumper.left                = (pack[56] & LT_BUMPER_LEFT);
  ret.light_bumper.front_left          = (pack[56] & LT_BUMPER_FRONT_LEFT);
  ret.light_bumper.center_left         = (pack[56] & LT_BUMPER_CENTER_LEFT);
  ret.light_bumper.center_right        = (pack[56] & LT_BUMPER_CENTER_RIGHT);
  ret.light_bumper.front_right         = (pack[56] & LT_BUMPER_FRONT_RIGHT);
  ret.light_bumper.right               = (pack[56] & LT_BUMPER_RIGHT);
  // Light Bumper Signal Strength
  ret.light_bumper.left_signal         = *((uint16*) (pack + 57));
  ret.light_bumper.front_left_signal   = *((uint16*) (pack + 59));
  ret.light_bumper.center_left_signal  = *((uint16*) (pack + 61));
  ret.light_bumper.center_right_signal = *((uint16*) (pack + 63));
  ret.light_bumper.front_right_signal  = *((uint16*) (pack + 65));
  ret.light_bumper.right_signal        = *((uint16*) (pack + 67));
  // Motor Current
  ret.motor_current.left_wheel         = *((int16*) (pack + 71));
  ret.motor_current.right_wheel        = *((int16*) (pack + 73));
  ret.motor_current.main_brush         = *((int16*) (pack + 75));
  ret.motor_current.side_brush         = *((int16*) (pack + 77));
  // Stasis
  ret.stasis                           =  pack[79];

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
