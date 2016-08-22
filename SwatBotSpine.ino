#define ERROR_STR "!! ERROR "
#define SERIAL_IN false // Debug - use serial STRICTLY for setting throttle

#define NUM_RC_CHANNELS 6
#define NUM_ESC_MOTORS 4

const int PITCH_MAX = 25;
const int ROLL_MAX = 25;
const int YAW_MAX = 5;

const String TYPE_X = "type_x"; // other types not implemented yet TYPE_H, TYPE_P(+)
const String UAV_TYPE = TYPE_X;

const int PIN_ESC_MOTOR[NUM_ESC_MOTORS] = {6,9,10,11}; 

// next variable names are controlled by rc lib :( -jkr
const uint8_t RC_Channel_Pin[NUM_RC_CHANNELS] = {A0,A1,A2,A3,3,5};
uint16_t RC_Channel_Value[NUM_RC_CHANNELS];

bool armed = false;

#include <SoftwareServo.h>
#include <PID_v1.h>
#include <Filters.h>
#include "PinChangeInt.h"
#include <RCLib.h> // needs to be after code above
#include "KalmanFilter.h"
#include "Channel.h"
#include "ESC.h"

#include "RemoteControl.h"
#include "ROSController.h"
#include "IMUController.h"
#include "StabilityController.h"
#include "ESCController.h"

RemoteControl* rc;
ROSController* ros;
IMUController* imu;
StabilityController *sc;
ESCController* ec;

void setup() {
  Serial.begin(115200);
  Serial.println("booting up...");

  SetRCInterrupts(); // for reading rc inputs / pinchangeinterrupt

  rc = new RemoteControl();
  rc->addChannel(new Channel(Channel::ELEVATOR, RC_Channel_Pin[0], 1150, 1850));
  rc->addChannel(new Channel(Channel::AILERON,  RC_Channel_Pin[1], 1150, 1850));
  rc->addChannel(new Channel(Channel::THROTTLE, RC_Channel_Pin[2], 1150, 1820));
  rc->addChannel(new Channel(Channel::RUDDER,   RC_Channel_Pin[3], 1150, 1850));
  if(RC_Channel_Pin[5])
    rc->addChannel(new Channel(Channel::AUX1, RC_Channel_Pin[4], 990, 2000));
  if(RC_Channel_Pin[6])
    rc->addChannel(new Channel(Channel::AUX2, RC_Channel_Pin[5], 990, 2000));

  ros = new ROSController();
  imu = new IMUController();
  sc = new StabilityController(rc, ros, imu);
  ec = new ESCController(rc, ros, sc);

  Serial.println("SWATBOT SETUP COMPLETE");
}

void loop() {
//  Serial.println("loop");

  rc->loop(); // read controls - RC overrides ROS
  ros->loop(); // read instructions from ros
  imu->loop(); // get current position
  sc->loop(); // desired RX/ROS input vs IMU input, calc new position
  ec->loop(); // send new position to esc
}
