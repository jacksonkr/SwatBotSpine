/**

Copyright © 2016 Jackson Rollins
Licensor: Jackson Rollins
Software: SwatBotSpin v0.1
Use Limitation: 10 users

License Grant. Licensor hereby grants to each recipient of the Software (“you”)
a non-exclusive, non-transferable, royalty-free and fully-paid-up license, under
all of the Licensor’s copyright and patent rights, to use, copy, distribute, 
prepare derivative works of, publicly perform and display the Software, subject 
to the Use Limitation and the conditions set forth below.

Use Limitation. The license granted above allows use by up to the number of users 
per entity set forth above (the “Use Limitation”). For determining the number of 
users, “you” includes all affiliates, meaning legal entities controlling, 
controlled by, or under common control with you. If you exceed the Use 
Limitation, your use is subject to payment of Licensor’s then-current list price 
for licenses.

Conditions. Redistribution in source code or other forms must include a copy of 
this license document to be provided in a reasonable manner. Any redistribution 
of the Software is only allowed subject to this license.

Trademarks. This license does not grant you any right in the trademarks, 
service marks, brand names or logos of Licensor.

DISCLAIMER. THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OR CONDITION, 
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. LICENSORS HEREBY DISCLAIM 
ALL LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE.
 
 */

#define ERROR_STR   "!! ERROR "
//#define ROS_C_ENABLED // only if we're using ROS

#define EEPROM_ADDRESS_IMU_GROUNDED   0 // the eeprom address wehre we store / retrieve imu grounded values
#define EEPROM_ADDRESS_IMU_FRONT      1 // the eeprom address where we store / retrieve imu front
#define NUM_RC_CHANNELS   6
#define NUM_ESC_MOTORS    4

const int PITCH_MAX = 10;
const int ROLL_MAX = 10;
const int YAW_MAX = 5;

/**
 * degrees front is off (front mathces orientation of writing on the board); 
 * 0 degress is facing to the right (east)
 * 90 degrees is neutral, everything is facing where it should be
 */
const char* TYPE_X = "type_x"; // other types not implemented yet TYPE_TRI, TYPE_H, TYPE_PLUS(+)
const char* UAV_TYPE = TYPE_X;

const int PIN_ESC_MOTOR[NUM_ESC_MOTORS] = {6,9,10,11}; 

// next variable names are controlled by rc lib :( -jkr
const uint8_t RC_Channel_Pin[NUM_RC_CHANNELS] = {A8,A9,A10,A11,A12,A13}; // <- for mega // {A0,A1,A2,A3,3,5}; // <- for uno -jkr
uint16_t RC_Channel_Value[NUM_RC_CHANNELS];

bool armed = false;

#include "helpers.h"
#include <SoftwareServo.h>
#include <PID_v1.h>
#include "PinChangeInt.h"
#include <RCLib.h> // needs to be after code above
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
#ifndef ROS_C_ENABLED
  Serial.begin(115200);
#endif
  Serial.println(F("booting up..."));

  SetRCInterrupts(); // for reading rc inputs / pinchangeinterrupt
  
  rc = new RemoteControl();
#ifdef ROS_C_ENABLED
  rosc = new ROSController();
#endif
  imu = new IMUController();
  sc = new StabilityController(rc, rosc, imu);
  ec = new ESCController(rc, rosc, sc);

  Serial.println(F("SWATBOT SETUP COMPLETE"));
}

void loop() {
//  Serial.println("loop");

  rc->loop(); // read controls - RC overrides ROS
#ifdef ROS_C_ENABLED
  rosc->loop(); // read instructions from ros
#endif
  imu->loop(); // get current position
  sc->loop(); // desired RX/ROS input vs IMU input, calc new position
  ec->loop(); // send new position to esc

  if(armed == false 
  && ec->getInitialized() == true 
  && imu->getInitialized() == true
  && (rc->isOn() == true 
#ifdef ROS_C_ENABLED
  || rosc->isOn() == true
#endif
  )) {
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
