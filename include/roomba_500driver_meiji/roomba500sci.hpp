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
/*
#include <stdexcept>
#include <string>

class RuntimeError : public std::exception {
  std::string message_;
  void concat(std::string message);

  void concat(const char* message);

  template <typename T> void concat(T arg) {
    message_ += std::to_string(arg);
  };
  template <typename T, typename... Args> void concat(T arg, Args... args) {
    concat(arg); concat(args...);
  };
public:
  template <typename... Args> RuntimeError(Args... args) {
    concat(args...);
  };
  virtual const char* what() const throw();
};
*/

#include <boost/thread.hpp>

#include "serial.hpp"
#include "roombaC2_types.hpp"

#include <roomba_500driver_meiji/RoombaCtrl.h>
#include <roomba_500driver_meiji/RoombaSensors.h>

typedef unsigned short uint16;
typedef unsigned char uint8;
typedef signed short int16;
typedef signed char int8;

const float COMMAND_WAIT = 0.01;    // [sec], this time is for Roomba 500 series
const int16 DEFAULT_VELOCITY = 200; // [mm/s]

namespace roombaC2 {
  class Roomba {
    Serial* comm_;
    roomba_500driver_meiji::RoombaCtrl ctrl_;
    roomba_500driver_meiji::RoombaSensors sensor_;

    boost::mutex ctrl_mutex_;
    boost::mutex sensor_mutex_;

    unsigned int enc_count_l_;
    unsigned int enc_count_r_;
    int d_enc_count_l_;
    int d_enc_count_r_;
    int d_pre_enc_l_;
    int d_pre_enc_r_;

    void convertState(const uint8 raw_sensors[80],
		      roomba_500driver_meiji::RoombaSensors &sensors);

  public:
    Roomba();
    ~Roomba();

    void init(int baud=B115200, const char* dev="/dev/ttyUSB0");

    void wakeup();
    void startup();
    void powerOff();
    void clean();
    void safe();
    void full();
    void spot();
    void max();
    void dock();

    void setMotorState(roombaC2::MOTOR_STATE_BITS state);
    void seekDock();

    void drive(int16 velocity, int16 radius);
    void drivePWM(int right_pwm,int left_pwm);
    void driveDirect(float velocity, float yawrate);

    void song(int song_number, int song_length);
    void playing(int song_number);

    int16 velToPWMRight(float velocity);
    int16 velToPWMLeft(float velocity);
    float velToPWM(float velocity);

    void sendOpCode(roombaC2::OPCODE, const uint8 *dataBytes=NULL, uint nDataBytes=0);
    void updateSensorState();
    void setTimestamp(ros::Time time);
    roomba_500driver_meiji::RoombaSensors getSensorState() const;
    void printSensorState();

    void setTravelDistance(short dist);
    void setTravelAngle(short angle);

    float getCtrlLinearX();
    float getCtrlAngleZ();

    int dEncoderRight(int max_delta=200);
    int dEncoderLeft(int max_delta=200);

    void sendCtrl(const roomba_500driver_meiji::RoombaCtrlConstPtr& msg);
  }; // class Roomba
}; // namespace roombaC2
#endif	// _ROOMBASCI_H
