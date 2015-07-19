#ifndef CREATE2_DRIVER_CREATE2SCI_HPP_
#define CREATE2_DRIVER_CREATE2SCI_HPP_

#include <create2_msgs/RoombaSensors.h>
#include <create2_msgs/RoombaCtrl.h>

#include "create2types.hpp"
#include "serial.hpp"

#include <boost/thread.hpp>
#include <boost/atomic.hpp>

typedef unsigned short uint16;
typedef unsigned char uint8;
typedef signed short int16;
typedef signed char int8;

namespace create2 {
  const float COMMAND_WAIT = 0.01;  // [sec]
  // These values are specific to iRobot Create2.
  // They are used to compute the travel distance and angle manually,
  // since the internal distance/angle value returned by roomba wrong.
  // https://robotics.stackexchange.com/questions/7062/create2-angle-packet-id-20
  const float N_TICKS        = 508.8; // per revolution.
  const float WHEEL_BASE     = 235.0; // [mm]
  const float WHEEL_DIAMETER = 72.00; // [mm]

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

  class Roomba {
    Serial* comm_;
    create2_msgs::RoombaCtrl ctrl_;
    create2_msgs::RoombaSensors sensor_;

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
		      create2_msgs::RoombaSensors &sensors);
  public:
    Roomba();
    ~Roomba();

    void init(int baud=B115200, const char* dev="/dev/ttyUSB0");

    void sendOpCode(create2::OPCODE opcode,
		    const uint8 *dataBytes=NULL, uint nDataBytes=0);
    void sendCtrl(const create2_msgs::RoombaCtrlConstPtr& msg);

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

    int16 velToPWMRight(float velocity);
    int16 velToPWMLeft(float velocity);
    float velToPWM(float velocity);

    create2_msgs::RoombaSensors getSensorState() const;

  }; // class Roomba
}; // namespace create2
#endif	// CREATE2_DRIVER_CREATE2SCI_HPP_
