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
#include "roombaC2_types.hpp"

#include <roomba_500driver_meiji/RoombaCtrl.h>
#include <roomba_500driver_meiji/RoombaSensors.h>

#include <boost/atomic.hpp>
#include <boost/thread/thread.hpp>

typedef unsigned short uint16;
typedef unsigned char uint8;
typedef signed short int16;
typedef signed char int8;

const float COMMAND_WAIT = 0.01;    // [sec], this time is for Roomba 500 series
const int16 DEFAULT_VELOCITY = 20; // [mm/s]

namespace roombaC2 {
  class Roomba {
    Serial* comm_;
    roomba_500driver_meiji::RoombaCtrl ctrl_;
    roomba_500driver_meiji::RoombaSensors sensor_;

    boost::mutex ctrl_mutex_;
    boost::mutex sensor_mutex_;

    void convertState(const uint8 raw_sensors[80],
		      roomba_500driver_meiji::RoombaSensors &sensors);

    // State manager
    boost::atomic<bool> stopStateManager_;
    boost::thread stateManager_;

    int dEncoderRight(int max_delta=200);
    int dEncoderLeft(int max_delta=200);

    void startStateUpdater();
    void updateSensorState();
    void updateState();

  public:
    Roomba();
    ~Roomba();

    void init(int baud=B115200, const char* dev="/dev/ttyUSB0");

    roomba_500driver_meiji::RoombaSensors getSensorState() const;

    void sendOpCode(roombaC2::OPCODE, const uint8 *dataBytes=NULL, uint nDataBytes=0) const;

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






    float getCtrlLinearX();
    float getCtrlAngleZ();

    void sendCtrl(const roomba_500driver_meiji::RoombaCtrlConstPtr& msg);
  }; // class Roomba
}; // namespace roombaC2
#endif	// _ROOMBASCI_H
