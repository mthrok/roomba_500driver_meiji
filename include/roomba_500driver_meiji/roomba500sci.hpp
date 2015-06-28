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

#include "serial.hpp"
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

const float COMMAND_WAIT=0.01;	 // [sec], this time is for Roomba 500 series
const short DEFAULT_VELOCITY=200; // [mm/s]

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
  // Refer to
  // [1]
  // "iRobot® Create® 2 Open Interface (OI) Specification based on the
  //  iRobot® Roomba® 600"
  // http://www.irobot.com/~/media/MainSite/PDFs/About/STEM/Create/create_2_Open_Interface_Spec.pdf
  // [2]
  // "iRobot® Create OPEN INTERFACE v.2"
  // http://www.irobot.com/filelibrary/pdfs/hrd/create/Create%20Open%20Interface_v2.pdf
  // [3]
  // "iRobot® Roomba® Serial Command Interface (SCI) Specification"
  // http://www.ecsl.cs.sunysb.edu/mint/Roomba_SCI_Spec_Manual.pdf

  enum OPCODE {
    OC_RESET = 7,
    OC_START = 128,
    OC_BAUD  = 129, // (1)

    OC_CONTROL = 130, // Only explained in [3]
    OC_SAFE    = 131,
    OC_FULL    = 132,
    OC_POWER   = 133,
    OC_SPOT    = 134,
    OC_CLEAN   = 135,
    OC_MAX     = 136,
    OC_DRIVE   = 137, // (+4 bytes)
    OC_MOTORS  = 138, // (+1 bytes)
    OC_LEDS    = 139, // (+3 bytes)

    OC_SONG          = 140, // (+2N+2 bytes)
    OC_PLAY          = 141, // (+1 bytes)
    OC_SENSORS       = 142, // (+1 bytes)
    OC_SEEK_DOCK     = 143,
    OC_PWM_MOTORS    = 144, // (+3 bytes)
    OC_DRIVE_DIRECT  = 145, // (+4 bytes)
    OC_DRIVE_PWM     = 146, // (+4 bytes)
    OC_STREAM        = 148, // (+N+1 bytes)
    OC_QUERY_LIST    = 149, // (+N+1 bytes)
    OC_TOGGLE_STREAM = 150, // (+1 bytes)

    OC_SCHEDULING_LEDS  = 162, // (+2 bytes)
    OC_DIGIT_LEDS_RAW   = 163, // (+4 bytes)
    OC_DIGIT_LEDS_ASCII = 164, // (+4 bytes)
    OC_BUTTONS          = 165, // (+1 bytes)
    OC_SCHEDULE         = 167, // (+15 bytes)
    OC_SET_DAYTIME      = 168, // (+3 bytes)

    OC_STOP = 173,
  };

  enum BAUD_CODE {
    BC_300 = 0,
    BC_600 = 1,
    BC_1200 = 2,
    BC_2400 = 3,
    BC_4800 = 4,
    BC_9600 = 5,
    BC_14400 = 6,
    BC_19200 = 7,
    BC_28800 = 8,
    BC_38400 = 9,
    BC_57600 =10,
    BC_115200 =11
  };

  enum DAY_BITS {
    DB_SUNDAY    = 0x01,
    DB_MONDAY    = 0x02,
    DB_TUESDAY   = 0x04,
    DB_WEDNESDAY = 0x08,
    DB_THURSDAY  = 0x10,
    DB_FRIDAY    = 0x20,
    DB_SATURDAY  = 0x40,
  };

  enum DAY_CODE {
    DC_SUNDAY = 0,
    DC_MONDAY = 1,
    DC_TUESDAY = 2,
    DC_WEDNESDAY = 3,
    DC_THURSDAY = 4,
    DC_FRIDAY = 5,
    DC_SATURDAY = 6
  };

  enum MOTOR_STATE_BITS {
    MB_SIDE_BRUSH     = 0x01,
    MB_VACUUM         = 0x02,
    MB_MAIN_BRUSH     = 0x04,
    MB_SIDE_BRUSH_CW  = 0x08,
    MB_MAIN_BRUSH_DIR = 0x10
  };

  enum LED_BITS {
    LB_CHECK_ROBOT = 0x08,
    LB_DOCK        = 0x04,
    LB_SPOT        = 0x02,
    LB_DEBRIS      = 0x01
  };

  enum WEEKDAY_LED_BITS {
    // TODO
  };

  enum SCHEDULING_LED_BITS {
    // TODO
  };

  enum DIGIT_N_BITS {
    // TODO
  };

  enum BUTTONS_BITS {
    BB_CLEAN    = 0x01,
    BB_SPOT     = 0x02,
    BB_DOCK     = 0x04,
    BB_MINUTE   = 0x08,
    BB_HOUR     = 0x10,
    BB_DAY      = 0x20,
    BB_SCHEDULE = 0x40,
    BB_CLOCK    = 0x80,
  };

  enum LED_ASCII {
    // TODO
  };

  enum SONG_NOTE {
    // TODO
  };

  enum PACKET_ID {
    GROUP_0 = 0,
    GROUP_1 = 1,
    GROUP_2 = 2,
    GROUP_3 = 3,
    GROUP_4 = 4,
    GROUP_5 = 5,
    GROUP_6 = 6,
    GROUP_101  = 101,
    GROUP_106  = 106,
    GROUP_107  = 107,
    ALL_PACKET = 100
  };

  enum PACKET_CODE {
    PC_0 = 0,
    PC_1 = 1,
    PC_2 = 2,
    PC_3 = 3,
  };

  enum BUMPS_WHEELDROPS {
    BUMP_RIGHT       = 0x01,
    BUMP_LEFT        = 0x02,
    WHEELDROP_RIGHT  = 0x04,
    WHEELDROP_LEFT   = 0x08
  };

  enum WALL {
    NO_WALL=0,
    WALL_SEEN=1
  };

  enum CLIRFF_LEFT {
    NO_CLIFF_LEFT = 0,
    CLIFF_LEFT    = 1
  };

  enum CLIRFF_FRONT_LEFT {
    NO_CLIFF_FRONT_LEFT = 0,
    CLIFF_FRONT_LEFT = 1
  };

  enum CLIRFF_FRONT_RIGHT {
    NO_CLIFF_FRONT_RIGHT = 0,
    CLIFF_FRONT_RIGHT = 1
  };

  enum CLIRFF_RIGHT {
    NO_CLIFF_RIGHT = 0,
    CLIFF_RIGHT = 1
  };

  enum VWALL {
    NO_VWALL = 0,
    VWALL_SEEN = 1
  };

  enum WHEEL_OVERCURRENT {
    SIDE_BRUSH = 0x01,
    MAIN_BRUSH = 0x04,
    RIGHT_WHEEL = 0x08,
    LEFT_WHEEL = 0x16
  };

  enum CHARGING_STATE_CODES {
    NOT_CHARGING      = 0,
    CHARGING_RECOVERY = 1,
    CHARGING          = 2,
    TRICKLE_CHARGING  = 3,
    WAITING           = 4,
    CHARGING_ERROR    = 5
  };

  enum CHARGIN_SOURCES_AVAILABLE {
    HOME_BASE        = 0x02,
    INTERNAL_CHARGER = 0x01
  };

  enum OI_MODE {
    OFF = 0,
    PASSIVE = 1,
    SAFE = 2,
    FULL = 3
  };

  enum LIGHT_BUMPER {
    LT_BUMBER_LEFT         = 0x01,
    LT_BUMBER_FRONT_LEFT   = 0x02,
    LT_BUMBER_CENTER_LEFT  = 0x04,
    LT_BUMBER_CENTER_RIGHT = 0x08,
    LT_BUMBER_FRONT_RIGHT  = 0x10,
    LT_BUMBER_RIGHT        = 0x20,
  };

  enum OVERCURRENT_BITS {
    OCB_SIDE_BRUSH  = 0x01,
    OCB_MAIN_BRUSH  = 0x02,
    OCB_RIGHT_WHEEL = 0x04,
    OCB_LEFT_WHEEL  = 0x08
  };
  /* Not found in Create2 manual. To be checked with older manuals
  enum MOTOR{MOTOR_ON=1, MOTOR_OFF=0};

  enum REMOTE_CC{NO_REMOTE=255};

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
  */

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

  void driveMotors(roombaSci::MOTOR_STATE_BITS state);
  void seekDock();

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
