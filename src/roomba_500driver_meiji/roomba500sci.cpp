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
#include <stdexcept>
#include <string>
/*
void RuntimeError::concat(std::string message) {
  message_ += message;
}

void RuntimeError::concat(const char* message) {
  message_ += std::string(message);
}

const char* RuntimeError::what() const throw () {
  return message_.c_str();
}
*/
#include <unistd.h>
#include <cmath>

using namespace roombaC2;
using namespace std;

typedef roomba_500driver_meiji::RoombaState RoombaState;
typedef roomba_500driver_meiji::RoombaCtrl RoombaCtrl;

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
  delete comm_;
}

void Roomba::init(int baud, const char* dev) {
  comm_ = new Serial(baud, dev, 80, 0);
  sleep(1);

  updateSensorState();
};

void Roomba::wakeup(void) {
  comm_ -> setRts(0);
  sleep_for_sec(0.1);
  comm_ -> setRts(1);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::startup(void) {
  sendOpCode(OC_START);
  sendOpCode(OC_CONTROL);
}

void Roomba::powerOff() {
  sendOpCode(OC_POWER);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::clean() {
  sendOpCode(OC_CLEAN);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::safe() {
  sendOpCode(OC_SAFE);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::full() {
  sendOpCode(OC_FULL);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::spot() {
  sendOpCode(OC_SPOT);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::max() {
  sendOpCode(OC_MAX);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::dock() {
  const uint8 seq[] = {OC_BUTTONS, BB_DOCK};
  comm_ -> write(seq,2);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::setMotorState(MOTOR_STATE_BITS state) {
  const uint8 seq[] = {OC_MOTORS, state};
  comm_ -> write(seq,2);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::seekDock() {
  const uint8 seq[] = {OC_SEEK_DOCK};
  comm_ -> write(seq,1);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::drive(short velocity, short radius) {
  uint8 vhi = (uint8)(velocity >> 8);
  uint8 vlo = (uint8)(velocity & 0xff);
  uint8 rhi = (uint8)(radius >> 8);
  uint8 rlo = (uint8)(radius & 0xff);

  const uint8 seq[] = {OC_DRIVE, vhi, vlo, rhi, rlo};
  comm_ -> write(seq, 5);
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
  comm_ -> write(seq, 5);
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
  comm_ -> write(seq,5);
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
  comm_ -> write(mode_seq, 1);
  sleep_for_sec(COMMAND_WAIT);

  const uint8 command_seq[] = {OC_SONG, song_number, song_length, 60, 126};
  comm_ -> write(command_seq,2*song_length+3);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::playing(int song_number) {
  const uint8 command_seq[] = {OC_PLAY, song_number};
  comm_ -> write(command_seq,2);
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::sendOpCode(OPCODE oc, const uint8* dataBytes, uint nDataBytes) {
  // Concatenate operation code and databytes
  uint nMsg = nDataBytes + 1;
  uint8 *message = new uint8[nMsg];
  message[0] = (uint8)oc;
  memcpy(message+1, dataBytes, nDataBytes);
  // Send message
  if (nMsg != comm_ -> write(message, nMsg)) {
    std::string err_msg(__func__); err_msg += ":Failed to send command.";
    throw std::runtime_error(err_msg.c_str());
  }
  sleep_for_sec(COMMAND_WAIT);
}

void Roomba::updateSensorState() {
  boost::mutex::scoped_lock(state_mutex_);
  uint8 raw_state[80];

  uint8 dataByte = ALL_PACKET;
  sendOpCode(OC_SENSORS, &dataByte, 1);
  if (80 == (comm_ -> read(raw_state, 80))) {
    convertState(raw_state, state_);
  } else {
    //std::string err_msg(__func__); err_msg += ":Failed to receive sensor state.";
    //throw std::runtime_error(err_msg.c_str());
  }
}

void Roomba::setTimestamp(ros::Time time) {
  state_.header.stamp = time;
}

RoombaState Roomba::getSensorState() const {
  return state_;
}

void Roomba::setTravelDistance(short dist) {
  boost::mutex::scoped_lock(state_mutex_);
  state_.travel.distance = dist;
}

void Roomba::setTravelAngle(short angle) {
  boost::mutex::scoped_lock(state_mutex_);
  state_.travel.angle = angle;
}

float Roomba::getCtrlLinearX() {
  return ctrl_.cntl.linear.x;
}

float Roomba::getCtrlAngleZ() {
  return ctrl_.cntl.angular.z;
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

void Roomba::sendCtrl(const roomba_500driver_meiji::RoombaCtrlConstPtr& msg) {
  boost::mutex::scoped_lock(ctrl_mutex_);
  ctrl_ = *msg;
  switch(msg->mode){
  case roomba_500driver_meiji::RoombaCtrl::SPOT:
    spot();
    break;
  case roomba_500driver_meiji::RoombaCtrl::SAFE:
    safe();
    break;
  case roomba_500driver_meiji::RoombaCtrl::CLEAN:
    clean();
    break;
  case roomba_500driver_meiji::RoombaCtrl::POWER:
    powerOff();
    break;
  case roomba_500driver_meiji::RoombaCtrl::WAKEUP:
    wakeup();
    startup();
    break;
  case roomba_500driver_meiji::RoombaCtrl::FULL:
    full();
    break;
  case roomba_500driver_meiji::RoombaCtrl::MAX:
    max();
    break;
  case roomba_500driver_meiji::RoombaCtrl::DOCK:
    dock();
    break;
  case roomba_500driver_meiji::RoombaCtrl::MOTORS:
    setMotorState((roombaC2::MOTOR_STATE_BITS)
		  (roombaC2::MB_MAIN_BRUSH |
		   roombaC2::MB_SIDE_BRUSH |
		   roombaC2::MB_VACUUM));
    break;
  case roomba_500driver_meiji::RoombaCtrl::MOTORS_OFF:
    setMotorState((roombaC2::MOTOR_STATE_BITS)(0));
    break;
  case roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT:
    driveDirect(msg->cntl.linear.x, msg->cntl.angular.z);
    break;
  case roomba_500driver_meiji::RoombaCtrl::DRIVE_PWM:
    drivePWM(msg->r_pwm, msg->l_pwm);
    break;
  case roomba_500driver_meiji::RoombaCtrl::SONG:
    safe();
    song(1,1);
    playing(1);
    break;
  case roomba_500driver_meiji::RoombaCtrl::DRIVE:
  default:
    drive(msg->velocity, msg->radius);
  }
}

void Roomba::convertState(const uint8 raw_state[80], RoombaState &state) {
  // Bumps and Wheel Drops
  state.bumps_wheeldrops.bump_right      = (raw_state[0] & BUMP_RIGHT);
  state.bumps_wheeldrops.bump_left       = (raw_state[0] & BUMP_LEFT);
  state.bumps_wheeldrops.wheeldrop_right = (raw_state[0] & WHEELDROP_RIGHT);
  state.bumps_wheeldrops.wheeldrop_left  = (raw_state[0] & WHEELDROP_LEFT);
  // Wall
  state.wall.wall                        = (raw_state[1] & 0x01);
  // Cliff
  state.cliff.left                       = (raw_state[2] & 0x01);
  state.cliff.front_left                 = (raw_state[3] & 0x01);
  state.cliff.front_right                = (raw_state[4] & 0x01);
  state.cliff.right                      = (raw_state[5] & 0x01);
  // Vertial Wall
  state.wall.vwall                       = (raw_state[6] & 0x01);
  // Wheel Overcurrents
  state.wheel_overcurrents.left_wheel    = (raw_state[7] & LEFT_WHEEL);
  state.wheel_overcurrents.right_wheel   = (raw_state[7] & RIGHT_WHEEL);
  state.wheel_overcurrents.main_brush    = (raw_state[7] & MAIN_BRUSH);
  state.wheel_overcurrents.side_brush    = (raw_state[7] & SIDE_BRUSH);
  // Dirt Detect
  state.dirt_detect                      = raw_state[8];
  // Unused Byte
  // raw_state[9] is unused byte
  // IR Opcodes
  state.ir_opcodes.omni                  = raw_state[10];
  state.ir_opcodes.left                  = raw_state[69]; // <- Attention
  state.ir_opcodes.right                 = raw_state[70]; // <- It's confusing.
  // Buttons
  state.button.clean                     = (raw_state[11] & BB_CLEAN);
  state.button.spot                      = (raw_state[11] & BB_SPOT);
  state.button.dock                      = (raw_state[11] & BB_DOCK);
  state.button.minute                    = (raw_state[11] & BB_MINUTE);
  state.button.hour                      = (raw_state[11] & BB_HOUR);
  state.button.day                       = (raw_state[11] & BB_DAY);
  state.button.schedule                  = (raw_state[11] & BB_SCHEDULE);
  state.button.clock                     = (raw_state[11] & BB_CLOCK);
  // Traveled distance and angle
  state.travel.distance                  = *((int16*) (raw_state + 12));
  state.travel.angle                     = *((int16*) (raw_state + 14));
  // Battery Charging state, Voltage, Current, Temperature, Charge, Capacity
  state.battery.charging_state           = raw_state[16];
  state.battery.voltage                  = *((uint16*) (raw_state + 17));
  state.battery.current                  = *((int16*)  (raw_state + 19));
  state.battery.temperature              = *((int8*)   (raw_state + 21));
  state.battery.charge                   = *((uint16*) (raw_state + 22));
  state.battery.capacity                 = *((uint16*) (raw_state + 24));
  // Wall Signal Strength
  state.wall.wall_signal                 = *((uint16*) (raw_state + 26));
  // Cliff Signal Strength
  state.cliff.left_signal                = *((uint16*) (raw_state + 28));
  state.cliff.front_left_signal          = *((uint16*) (raw_state + 30));
  state.cliff.front_right_signal         = *((uint16*) (raw_state + 32));
  state.cliff.right_signal               = *((uint16*) (raw_state + 34));
  // Unused Byte
  // raw_state[36, 37, 38] is unused byte
  // Charging sources availability
  state.charging_source.home_base        = (raw_state[39] & HOME_BASE);
  state.charging_source.internal_charger = (raw_state[39] & INTERNAL_CHARGER);
  // OI Mode
  state.oi_mode                          = raw_state[40];
  // Song
  state.song.number                      = raw_state[41];
  state.song.playing                     = raw_state[42];
  // #Stream raw_stateets
  state.stream_packets                   = raw_state[43];
  // Requested Velocity and radius
  state.request.velocity                 = *((int16*) (raw_state + 44));
  state.request.radius                   = *((int16*) (raw_state + 46));
  state.request.right_velocity           = *((int16*) (raw_state + 48));
  state.request.left_velocity            = *((int16*) (raw_state + 50));
  // Encoder counts
  state.encoder_counts.left              = *((uint16*) (raw_state + 52));
  state.encoder_counts.right             = *((uint16*) (raw_state + 54));
  // Light Bumper
  state.light_bumper.left                = (raw_state[56] & LT_BUMPER_LEFT);
  state.light_bumper.front_left          = (raw_state[56] & LT_BUMPER_FRONT_LEFT);
  state.light_bumper.center_left         = (raw_state[56] & LT_BUMPER_CENTER_LEFT);
  state.light_bumper.center_right        = (raw_state[56] & LT_BUMPER_CENTER_RIGHT);
  state.light_bumper.front_right         = (raw_state[56] & LT_BUMPER_FRONT_RIGHT);
  state.light_bumper.right               = (raw_state[56] & LT_BUMPER_RIGHT);
  // Light Bumper Signal Strength
  state.light_bumper.left_signal         = *((uint16*) (raw_state + 57));
  state.light_bumper.front_left_signal   = *((uint16*) (raw_state + 59));
  state.light_bumper.center_left_signal  = *((uint16*) (raw_state + 61));
  state.light_bumper.center_right_signal = *((uint16*) (raw_state + 63));
  state.light_bumper.front_right_signal  = *((uint16*) (raw_state + 65));
  state.light_bumper.right_signal        = *((uint16*) (raw_state + 67));
  // Motor Current
  state.motor_current.left_wheel         = *((int16*) (raw_state + 71));
  state.motor_current.right_wheel        = *((int16*) (raw_state + 73));
  state.motor_current.main_brush         = *((int16*) (raw_state + 75));
  state.motor_current.side_brush         = *((int16*) (raw_state + 77));
  // Stasis
  state.stasis                           =  raw_state[79];

  // Update internal encoder counts
  if(std::abs((int)state.encoder_counts.right - (int)enc_count_r_) >= 60000) {
    if(state.encoder_counts.right > enc_count_r_)
      d_enc_count_r_ = -65535 - enc_count_r_ + state.encoder_counts.right;
    else
      d_enc_count_r_ = 65535 - enc_count_r_ + state.encoder_counts.right;
  } else {
    d_enc_count_r_ = state.encoder_counts.right - enc_count_r_;
  }

  if(std::abs((int)state.encoder_counts.left - (int)enc_count_l_) >= 60000) {
    if(state.encoder_counts.left > enc_count_l_)
      d_enc_count_l_ = -65535-enc_count_l_+state.encoder_counts.left;
    else
      d_enc_count_l_ = 65535-enc_count_l_+state.encoder_counts.left;
  } else {
    d_enc_count_l_ = state.encoder_counts.left - enc_count_l_;
  }

  enc_count_r_ = state.encoder_counts.right;
  enc_count_l_ = state.encoder_counts.left;
}

void Roomba::printSensorState() {
  cout << "\n\n-------------------" << endl
       << "Bumps:\n  "
       << (state_.bumps_wheeldrops.bump_right ? "Right, " : "       ")
       << (state_.bumps_wheeldrops.bump_left  ? "Left" : "") << endl
       << "Wheeldrops:\n "
       << (state_.bumps_wheeldrops.wheeldrop_right ? "Right, " : "       ")
       << (state_.bumps_wheeldrops.wheeldrop_left  ? "Left" : "") << endl
       << "Wall:\n  "
       << (state_.wall.wall ? "Wall, " : "      ")
       << (state_.wall.vwall ? "Virtual Wall" : "") << endl
       << "Cliff:\n  "
       << (state_.cliff.left ? "Left, " : "")
       << (state_.cliff.front_left ? "Front Left, " : "")
       << (state_.cliff.right ? "Right, " : "")
       << (state_.cliff.front_right ? "Front Right " : "") << endl
       << "Wheel Overcurrent:\n  "
       << (state_.wheel_overcurrents.side_brush ? "Side Brush, " : "")
       << (state_.wheel_overcurrents.main_brush ? "Main Brush, " : "")
       << (state_.wheel_overcurrents.right_wheel ? "Right Wheel, " : "")
       << (state_.wheel_overcurrents.left_wheel ? "Left Wheel" : "") << endl
       << "Dirt Detection:  " << (int)state_.dirt_detect << endl
       << "IR Character:"
       << "\n  Right: " << (int)state_.ir_opcodes.right
       << "\n  Left: "  << (int)state_.ir_opcodes.left
       << "\n  Omni: "  << (int)state_.ir_opcodes.omni << endl
       << "Buttons:\n  "
       << (state_.button.clean    ? "Clean, " : "")
       << (state_.button.spot     ? "Spot, " : "")
       << (state_.button.dock     ? "Dock, " : "")
       << (state_.button.minute   ? "Minute, " : "")
       << (state_.button.hour     ? "Hour, " : "")
       << (state_.button.day      ? "Day, " : "")
       << (state_.button.clock    ? "Clock, " : "")
       << (state_.button.schedule ? "Schedule" : "") << endl
       << "Traveled (From the last acquisition.):"
       << "\n  Distance: " << state_.travel.distance << " [mm]"
       << "\n  Angle: " << state_.travel.angle << " [rad]" << endl
       << "Battery:"
       << "\n  Charging State: " << (uint)state_.battery.charging_state
       << "\n  Voltage: " << state_.battery.voltage
       << "\n  Current: " << state_.battery.current
       << "\n  Temperature: " << (int) state_.battery.temperature
       << "\n  Ramains: "
       << state_.battery.charge << " / " << state_.battery.capacity
       << " (" << 100.0f * state_.battery.charge / state_.battery.capacity << "[%])"
       << endl
       << "Charge Source:\n  "
       << (state_.charging_source.home_base ? "Home Base, " : "")
       << (state_.charging_source.internal_charger ? "Internal Charger." : "")
       << endl
       << "OI Mode : " << (int)state_.oi_mode << endl
       << "Song:"
       << "\n  Numer:   " << (int)state_.song.number
       << "\n  Playing: " << (int)state_.song.playing << endl
       << "Request:"
       << "\n  Velocity: " << state_.request.velocity
       << "\n  Radius: " << state_.request.radius
       << "\n  Right Velocity: " << state_.request.right_velocity
       << "\n  Left Velocity: " << state_.request.left_velocity << endl
       << "Encoder Count:"
       << "\n  L / R: " << state_.encoder_counts.right
       << " / " << state_.encoder_counts.left << endl
       << "Light Bumper: "
       << "\n  Left:         "
       << (state_.light_bumper.left ? "    Hit" : "Not Hit")
       << ", (" << state_.light_bumper.left_signal << ")"
       << "\n  Right:        "
       << (state_.light_bumper.right ? "    Hit" : "Not Hit")
       << ", (" << state_.light_bumper.right_signal << ")"
       << "\n  Front Left:   "
       << (state_.light_bumper.front_left ? "    Hit" : "Not Hit")
       << ", (" << state_.light_bumper.front_left_signal << ")"
       << "\n  Front Right:  "
       << (state_.light_bumper.front_right ? "    Hit" : "Not Hit")
       << ", (" << state_.light_bumper.front_right_signal << ")"
       << "\n  Center Left:  "
       << (state_.light_bumper.center_left ? "    Hit" : "Not Hit")
       << ", (" << state_.light_bumper.center_left_signal << ")"
       << "\n  Center Right: "
       << (state_.light_bumper.center_right ? "    Hit" : "Not Hit")
       << ", (" << state_.light_bumper.center_right_signal << ")" << endl
       << "Motor Current:"
       << "\n  Left Wheel: " << state_.motor_current.left_wheel
       << "\n  Right Wheel: " << state_.motor_current.right_wheel
       << "\n  Main Brush: " << state_.motor_current.main_brush
       << "\n  Side Brush: " << state_.motor_current.side_brush << endl
       << "Stasis: " << (state_.stasis ? "Forward" : "Not Forward") << endl;
}
