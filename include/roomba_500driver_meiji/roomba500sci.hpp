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
#include <roomba_500driver_meiji/Roomba500State.h>

namespace roombaC2 {
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

  // Main Operation Codes
  enum OPCODE {
    OC_RESET = 7,
    OC_START = 128,
    OC_BAUD  = 129, // (1)

    OC_CONTROL = 130, // Only found in ref.[3], not in [1] and [2]
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
  // Data bytes for operation codes
  enum BAUD_CODE {
    BC_300    = 0,
    BC_600    = 1,
    BC_1200   = 2,
    BC_2400   = 3,
    BC_4800   = 4,
    BC_9600   = 5,
    BC_14400  = 6,
    BC_19200  = 7,
    BC_28800  = 8,
    BC_38400  = 9,
    BC_57600  = 10,
    BC_115200 = 11
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
    DC_SUNDAY    = 0,
    DC_MONDAY    = 1,
    DC_TUESDAY   = 2,
    DC_WEDNESDAY = 3,
    DC_THURSDAY  = 4,
    DC_FRIDAY    = 5,
    DC_SATURDAY  = 6
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
  enum SCHEDULING_LED_BITS {
    SLB_COLON    = 0x01,
    SLB_PM       = 0x02,
    SLB_AM       = 0x04,
    SLB_CLOCK    = 0x08,
    SLB_SCHEDULE = 0x10,
  };
  enum DIGIT_N_BITS {
    DNB_A = 0x01,
    DNB_B = 0x02,
    DNB_C = 0x04,
    DNB_D = 0x08,
    DNB_E = 0x10,
    DNB_F = 0x20,
    DNB_G = 0x40
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
  enum LED_ASCII_CODE {
    LAC_SPACE = 32,
    LAC_EXCLAMATION,
    LAC_DOUBLE_QUOTE,
    LAC_SHARP,
    LAC_PERCENT = 37,
    LAC_AMPERSAND,
    LAC_RIGHT_SINGLE_QUOTE,
    LAC_COMMA = 44,
    LAC_HYPHEN,
    LAC_PERIOD,
    LAC_SLASH ,
    LAC_0,
    LAC_1,
    LAC_2,
    LAC_3,
    LAC_4,
    LAC_5,
    LAC_6,
    LAC_7,
    LAC_8,
    LAC_9,
    LAC_COLON,
    LAC_SEMICOLON,
    LAC_INV_EXCLAMATION,
    LAC_EQUAL,
    LAC_INV_QUESTION,
    LAC_QUESTION,
    LAC_A = 65,
    LAC_B,
    LAC_C,
    LAC_D,
    LAC_E,
    LAC_F,
    LAC_G,
    LAC_H,
    LAC_I,
    LAC_J,
    LAC_K,
    LAC_L,
    LAC_M,
    LAC_N,
    LAC_O,
    LAC_P,
    LAC_Q,
    LAC_R,
    LAC_S,
    LAC_T,
    LAC_U,
    LAC_V,
    LAC_W,
    LAC_X,
    LAC_Y,
    LAC_Z,
    LAC_OPEN_BRACKET,
    LAC_BACKSLASH,
    LAC_CLOSE_BRACKET,
    LAC_CIRCUMFLEX,
    LAC_UNDERSCORE,
    LAC_LEFT_SINGLE_QUOTE,
    LAC_a,
    LAC_b,
    LAC_c,
    LAC_d,
    LAC_e,
    LAC_f,
    LAC_g,
    LAC_h,
    LAC_i,
    LAC_j,
    LAC_k,
    LAC_l,
    LAC_m,
    LAC_n,
    LAC_o,
    LAC_p,
    LAC_q,
    LAC_r,
    LAC_s,
    LAC_t,
    LAC_u,
    LAC_v,
    LAC_w,
    LAC_x,
    LAC_y,
    LAC_z,
    LAC_LEFT_BRACE,
    LAC_DASH,
    LAC_RIGHT_BRACE,
    LAC_TILDE
  };
  enum SONG_NOTE {
    NOTE_G1  = 31,
    NOTE_GS1,
    NOTE_A1,
    NOTE_AS1,
    NOTE_B1,
    NOTE_C1,
    NOTE_CS1,
    NOTE_D1,
    NOTE_DS1,
    NOTE_E1,
    NOTE_F1,
    NOTE_FS1,
    NOTE_G2 = 43,
    NOTE_GS2,
    NOTE_A2,
    NOTE_AS2,
    NOTE_B2,
    NOTE_C2,
    NOTE_CS2,
    NOTE_D2,
    NOTE_DS2,
    NOTE_E2,
    NOTE_F2,
    NOTE_FS2,
    NOTE_G3 = 55,
    NOTE_GS3,
    NOTE_A3,
    NOTE_AS3,
    NOTE_B3,
    NOTE_C3,
    NOTE_CS3,
    NOTE_D3,
    NOTE_DS3,
    NOTE_E3,
    NOTE_F3,
    NOTE_FS3,
    NOTE_G4 = 67,
    NOTE_GS4,
    NOTE_A4,
    NOTE_AS4,
    NOTE_B4,
    NOTE_C4,
    NOTE_CS4,
    NOTE_D4,
    NOTE_DS4,
    NOTE_E4,
    NOTE_F4,
    NOTE_FS4,
    NOTE_G5 = 79,
    NOTE_GS5,
    NOTE_A5,
    NOTE_AS5,
    NOTE_B5,
    NOTE_C5,
    NOTE_CS5,
    NOTE_D5,
    NOTE_DS5,
    NOTE_E5,
    NOTE_F5,
    NOTE_FS5,
    NOTE_G6 = 91,
    NOTE_GS6,
    NOTE_A6,
    NOTE_AS6,
    NOTE_B6,
    NOTE_C6,
    NOTE_CS6,
    NOTE_D6,
    NOTE_DS6,
    NOTE_E6,
    NOTE_F6,
    NOTE_FS6,
    NOTE_G7 = 103,
    NOTE_GS7,
    NOTE_A7,
    NOTE_AS7,
    NOTE_B7,
  };
  // SENSOR PACKET DIFINITIONS
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
  enum BUMPS_WHEELDROPS {
    BUMP_RIGHT       = 0x01,
    BUMP_LEFT        = 0x02,
    WHEELDROP_RIGHT  = 0x04,
    WHEELDROP_LEFT   = 0x08
  };
  enum WHEEL_OVERCURRENT {
    SIDE_BRUSH  = 0x01,
    MAIN_BRUSH  = 0x04,
    RIGHT_WHEEL = 0x08,
    LEFT_WHEEL  = 0x16
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
    OFF     = 0,
    PASSIVE = 1,
    SAFE    = 2,
    FULL    = 3
  };
  enum LIGHT_BUMPER {
    LT_BUMPER_LEFT         = 0x01,
    LT_BUMPER_FRONT_LEFT   = 0x02,
    LT_BUMPER_CENTER_LEFT  = 0x04,
    LT_BUMPER_CENTER_RIGHT = 0x08,
    LT_BUMPER_FRONT_RIGHT  = 0x10,
    LT_BUMPER_RIGHT        = 0x20,
  };
  enum OVERCURRENT_BITS {
    OCB_SIDE_BRUSH  = 0x01,
    OCB_MAIN_BRUSH  = 0x02,
    OCB_RIGHT_WHEEL = 0x04,
    OCB_LEFT_WHEEL  = 0x08
  };

  const float COMMAND_WAIT = 0.01;    // [sec], this time is for Roomba 500 series
  const short DEFAULT_VELOCITY = 200; // [mm/s]


  class Roomba {
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
    Roomba();
    ~Roomba();
    void init(int baud=B115200, const char* dev="/dev/ttyUSB0");

    void wakeup(void);
    void startup();
    void powerOff();
    void clean();
    void safe();
    void full();
    void spot();
    void max();
    void dock();

    void driveMotors(roombaC2::MOTOR_STATE_BITS state);
    void seekDock();

    void drive(short velocity, short radius);
    void driveDirect(float velocity, float yawrate);
    void drivePWM(int right_pwm,int left_pwm);

    void song(int song_number, int song_length);
    void playing(int song_number);

    short velToPWMRight(float velocity);
    short velToPWMLeft(float velocity);
    float velToPWM(float velocity);

    int sendOPCODE(roombaC2::OPCODE);
    int getSensors(roomba_500driver_meiji::Roomba500State& sensor);

    int dEncoderRight(int max_delta=200);

    int dEncoderLeft(int max_delta=200);
  }; // class Roomba
}; // namespace roombaC2
#endif	// _ROOMBASCI_H
