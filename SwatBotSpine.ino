#define ERROR_STR   "!! ERROR "
#define SERIAL_IN   false // Debug - use serial STRICTLY for setting throttle

#define EEPROM_ADDRESS_IMU_GROUNDED   0
#define EEPROM_ADDRESS_IMU_FRONT      1
#define NUM_RC_CHANNELS   6
#define NUM_ESC_MOTORS    4

const int PITCH_MAX = 10;
const int ROLL_MAX = 10;
const int YAW_MAX = 5;

const int FRONT = 90; // degrees front is off
const char* TYPE_X = "type_x"; // other types not implemented yet TYPE_H, TYPE_P(+)
const char* UAV_TYPE = TYPE_X;

const int PIN_ESC_MOTOR[NUM_ESC_MOTORS] = {6,9,10,11}; 

// next variable names are controlled by rc lib :( -jkr
const uint8_t RC_Channel_Pin[NUM_RC_CHANNELS] = {A0,A1,A2,A3,3,5};
uint16_t RC_Channel_Value[NUM_RC_CHANNELS];

bool armed = false;

// this isn't working when it's in helpers.cpp ??
void arrayShift(double* arr, double val, int len) {
  memmove(&arr[1], &arr[0], (len-1)*sizeof(double));
  arr[0] = val;
}

#include "helpers.cpp"
#include <SoftwareServo.h>
#include <PID_v1.h>
#include <Filters.h>
#include "PinChangeInt.h"
#include <RCLib.h> // needs to be after code above
#include <KalmanFilter.h>
#include "Channel.h"
#include "ESC.h"

#include "RemoteControl.h"
#include "ROSController.h"
#include "IMUController.h"
#include "StabilityController.h"
#include "ESCController.h"

RemoteControl* rc;
ROSController* rosc;
IMUController* imu;
StabilityController *sc;
ESCController* ec;

void setup() {
  Serial.begin(115200);
  Serial.println(F("booting up..."));

  SetRCInterrupts(); // for reading rc inputs / pinchangeinterrupt

  rc = new RemoteControl();
  rosc = new ROSController();
  imu = new IMUController();
  sc = new StabilityController(rc, rosc, imu);
  ec = new ESCController(rc, rosc, sc);

  Serial.println(F("SWATBOT SETUP COMPLETE"));
}

void loop() {
//  Serial.println("loop");

  rc->loop(); // read controls - RC overrides ROS
  rosc->loop(); // read instructions from ros
  imu->loop(); // get current position
  sc->loop(); // desired RX/ROS input vs IMU input, calc new position
  ec->loop(); // send new position to esc

  if(armed == false 
  && ec->getInitialized() == true 
  && imu->getInitialized() == true
  && (rc->isOn() == true || rosc->isOn() == true)) {
    // watch for arming sequence
    armed = true;
    Serial.println(F("ARMED!"));
  } else if(armed == true
  && rc->isOn() == false 
  && rosc->isOn() == false) {
    armed = false;
    Serial.println(F("DISARMED!"));
  }
}
