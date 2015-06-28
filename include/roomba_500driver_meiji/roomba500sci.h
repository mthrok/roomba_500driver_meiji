//---------------------------< /-/ AMSL /-/ >------------------------------
/**
 * file         :       roomba500sci.h
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

#ifndef _ROOMBASCI_H
#define _ROOMBASCI_H

#include "serial.h"
#include <sys/time.h>
#include <unistd.h>

#include <cmath>
#include <roomba_500driver_meiji/Roomba500State.h>

class Timer {
public:
  void sleep(float sec){
    long usec=(long)(sec*1000000);
    usleep(usec);
  }
};

const float COMMAND_WAIT=0.01;	 // sec, this time is for Roomba 500 series
const short DEFAULT_VELOCITY=200; // mm/s

// DRIVE Special codes
const short STRAIGHT_RADIUS=0x8000;
const short TURN_CLOCK=-1;
const short TURN_CNT_CLOCK=1;

class roombaSci {
 protected:
  int receive(void);
  int receive(unsigned char* pack, int byte);

  void packetToStruct(roomba_500driver_meiji::Roomba500State& ret,
		      const unsigned char* pack);
  Serial* ser_;
  unsigned char packet_[80];
  unsigned int enc_count_l_;
  unsigned int enc_count_r_;
  int d_enc_count_l_;
  int d_enc_count_r_;
  int d_pre_enc_l_;
  int d_pre_enc_r_;
 public:
  enum PACKET_ID{
    GROUP_0 = 0,
    GROUP_1 = 1,
    GROUP_2 = 2,
    GROUP_3 = 3,
    GROUP_4 = 4,
    GROUP_5 = 5,
    GROUP_6 = 6,
    GROUP_101 = 101,
    GROUP_106 = 106,
    GROUP_107 = 107,
    ALL_PACKET = 100};

  enum OPCODE {
    OC_START              = 128,
    OC_BAUD               = 129,
    OC_CONTROL            = 130,
    OC_SAFE               = 131,
    OC_FULL               = 132,
    OC_POWER              = 133,
    OC_SPOT               = 134,
    OC_CLEAN              = 135,
    OC_MAX                = 136,
    OC_DRIVE              = 137,
    OC_DRIVE_DIRECT       = 145,
    OC_DRIVE_PWM          = 146,
    OC_MOTORS             = 138,
    OC_LEDS               = 139,
    OC_SONG               = 140,
    OC_PLAY               = 141,
    OC_SENSORS            = 142,
    OC_QUERY_LIST         = 149,
    OC_STREAM             = 148,
    OC_PAUSE_STREAM       = 150,
    OC_FORCE_SEEKING_DOCK = 143,
    OC_BUTTONS            = 165
  };

  enum BAUD_CODE {
    BC_300      = 0,
    BC_600      = 1,
    BC_1200     = 2,
    BC_2400     = 3,
    BC_4800     = 4,
    BC_9600     = 5,
    BC_14400    = 6,
    BC_19200    = 7,
    BC_28800    = 8,
    BC_38400    = 9,
    BC_57600    =10,
    BC_115200   =11
  };

  enum MOTOR_BITS {
    MB_MAIN_BRUSH = 0x04,
    MB_VACUUM     = 0x02,
    MB_SIDE_BRUSH = 0x01
  };

  enum LED_BITS {
  };

  enum PACKET_CODE {
    PC_0 = 0,
    PC_1 = 1,
    PC_2 = 2,
    PC_3 = 3,
  };

  enum BUTTONS_BIT{
    BUTTON_CLEAN=0x01,
    BUTTON_SPOT=0x02,
    BUTTON_DOCK=0x04,
    BUTTON_MINUTE=0x08,
    BUTTON_HOUR=0x10,
    BUTTON_DAY=0x20,
    BUTTON_SCHEDULE=0x40,
    BUTTON_CLOCK=0x80,
  };

  enum MOTOR{MOTOR_ON=1, MOTOR_OFF=0};
  enum WALL{ NO_WALL=0, WALL_SEEN=1};
  enum CLIRFF{ NO_CLIFF=0, CLIFF=1};
  enum VWALL{NO_VWALL=0, VWALL_SEEN=1};
  enum OVER_CUR{NO_OVER_C=0, OVER_C=1};
  enum REMOTE_CC{NO_REMOTE=255};

  enum BUMPS_WHEELDROPS {
    WHEELDROP_CASTER = 0x10,
    WHEELDROP_LEFT   = 0x08,
    WHEELDROP_RIGHT  = 0x04,
    BUMP_LEFT        = 0x02,
    BUMP_RIGHT       = 0x01
  };

  enum MOTOR_OVERCURRENTS {
    DRIVE_LEFT  = 0x10,
    DRIVE_RIGHT = 0x08,
    MAIN_BRUSH  = 0x04,
    VACUUM      = 0x02,
    SIDE_BRUSH  = 0x01
  };

  enum BUTTONS {
    B_POWER = 0x08,
    B_SPOT  = 0x04,
    B_CLEAN = 0x02,
    B_MAX   = 0x01
  };

  enum CHARGING_STATE_CODES {
    NOT_CHARGING      = 0,
    CHARGING_RECOVERY = 1,
    CHARGING          = 2,
    TRICKLE_CHARGING  = 3,
    WAITING           = 4,
    CHARGING_ERROR    = 5
  };

  roombaSci(int baud=B115200, const char* dev="/dev/ttyUSB0");
  ~roombaSci();

  void wakeup(void);
  void startup();
  void powerOff();
  void clean();
  void safe();
  void full();
  void spot();
  void max();
  void dock();

  void driveMotors(roombaSci::MOTOR_BITS state);
  void forceSeekingDock();

  void drive(short velocity, short radius);
  void driveDirect(float velocity, float yawrate);
  void drivePWM(int right_pwm,int left_pwm);

  void song(int song_number, int song_length);
  void playing(int song_number);

  short velToPWMRight(float velocity);
  short velToPWMLeft(float velocity);
  float velToPWM(float velocity);

  int sendOPCODE(roombaSci::OPCODE);
  int getSensors();
  int getSensors(roomba_500driver_meiji::Roomba500State& sensor);

  int dEncoderRight(int max_delta=200){
    d_enc_count_r_=std::max(-max_delta,d_enc_count_r_);
    d_enc_count_r_=std::min(max_delta,d_enc_count_r_);
    return d_enc_count_r_;
  }

  int dEncoderLeft(int max_delta=200){
    d_enc_count_l_=std::max(-max_delta,d_enc_count_l_);
    d_enc_count_l_=std::min(max_delta,d_enc_count_l_);
    return d_enc_count_l_;
  }

  Timer* time_;
}; // class roombaSci
#endif	// _ROOMBASCI_H
