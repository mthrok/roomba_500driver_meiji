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

#include <roomba_500driver_meiji/RoombaSensors.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>
#include "roombaC2_types.hpp"
#include "serial.hpp"

#include <boost/thread.hpp>
#include <boost/atomic.hpp>

typedef unsigned short uint16;
typedef unsigned char uint8;
typedef signed short int16;
typedef signed char int8;

namespace roombaC2 {
  class State {
    float x_, y_, theta_, v_, w_;
    boost::mutex state_mutex_;
  public:
    void setXY(const float x, const float y);
    void setTheta(const float theta);
    void setVW(const float v, const float w);
    float getX() const;
    float getY() const;
    float getTheta() const;
    float getV() const;
    float getW() const;
    void updatePose(const float distance, const float angle);
    void updateSpeed(const float v, const float w);
  };

  // These values are specific to iRobot Create2.
  // They are used to compute the travel distance and angle manually,
  // since the internal distance/angle value returned by roomba wrong.
  // https://robotics.stackexchange.com/questions/7062/create2-angle-packet-id-20
  const float N_TICKS        = 508.8; // per revolution.
  const float WHEEL_BASE     = 235.0; // [mm]
  const float WHEEL_DIAMETER = 72.00; // [mm]

  const float COMMAND_WAIT = 0.01; // [sec]
  class Roomba {
    Serial* comm_;
    roomba_500driver_meiji::RoombaCtrl ctrl_;
    roomba_500driver_meiji::RoombaSensors sensor_;

    boost::mutex ctrl_mutex_;
    boost::mutex sensor_mutex_;

    // Global status
    State currentState_;
    State requestState_;

    // For state managing
    boost::atomic<bool> stopStateManager_;
    boost::thread stateManager_;

    void startStateManager();
    void updateSensorState();
    void updateRoombaState();

    void convertState(const uint8 raw_sensors[80],
		      roomba_500driver_meiji::RoombaSensors &sensors);
  public:
    Roomba();
    ~Roomba();

    void init(int baud=B115200, const char* dev="/dev/ttyUSB0");

    void sendOpCode(roombaC2::OPCODE opcode,
		    const uint8 *dataBytes=NULL, uint nDataBytes=0);
    void sendCtrl(const roomba_500driver_meiji::RoombaCtrlConstPtr& msg);

    void wakeup();
    void startup();
    void powerOff();
    void clean();
    void safe();
    void full();
    void spot();
    void max();
    void dock();

    void setMotorState(const uint8 state);
    void seekDock();

    void drive(int16 velocity, int16 radius);
    void drivePWM(int right_pwm,int left_pwm);
    void driveDirect(float velocity, float yawrate);

    void song(int song_number, int song_length);
    void playing(int song_number);

    int16 velToPWMRight(float velocity);
    int16 velToPWMLeft(float velocity);
    float velToPWM(float velocity);

    roomba_500driver_meiji::RoombaSensors getSensorState() const;
    void printSensorState();

    void setTravelDistance(short dist);
    void setTravelAngle(short angle);

    float getCtrlLinearX();
    float getCtrlAngleZ();

    int dEncoderRight(int max_delta=200);
    int dEncoderLeft(int max_delta=200);


  }; // class Roomba
}; // namespace roombaC2
#endif	// _ROOMBASCI_H
