#define ERROR_STR "!! ERROR"
#define NUM_RC_CHANNELS 4

// sensors replaced by rgbd odometry (for now)
//const int PIN_USENSOR_ABOVE_TRIG = -1;
//const int PIN_USENSOR_ABOVE_ECHO = -1;
//const int PIN_USENSOR_BELOW_TRIG = 12;
//const int PIN_USENSOR_BELOW_ECHO = 11; // needs to be PWM for pulsein ?
const int PIN_CH1_IN = 2;
const int PIN_CH2_IN = 4;
const int PIN_CH3_IN = 7;
const int PIN_CH4_IN = 8;
//const int PIN_CH5 = 6;
//const int PIN_CH6 = 7;
const int PIN_CH1_OUT = 3;
const int PIN_CH2_OUT = 5;
const int PIN_CH3_OUT = 6;
const int PIN_CH4_OUT = 9;

/** pin change interrupt for rc
 *  Leveraging this for the prototype, will rewrite later -jkr
 */
const uint8_t RC_Channel_Pin[NUM_RC_CHANNELS]={ 
PIN_CH1_IN, 
PIN_CH2_IN, 
PIN_CH3_IN, 
PIN_CH4_IN,
//PIN_CH5_IN,
//PIN_CH6_IN,
};
uint16_t RC_Channel_Value[NUM_RC_CHANNELS];

#include "PinChangeInt.h"
#include <RCLib.h> // needs to be after code above
#include "KalmanFilter.h"
//#include "UltrasonicSensor.h"
#include "Channel.h"

#include "RemoteControl.h"
#include <PID_v1.h>
#include <Filters.h>

// using rtabmap graph for drone height
//double floorInput, floorOutput;
//double floorSetpoint = 304.8 * 4; // mm_per_ft * feet; should be changeable by ros
//PID floorPID(&floorInput, &floorOutput, &floorSetpoint, 0.078, 0.196, 0.038, DIRECT); // in, out, target, P, I, D, direction (direct/reverse)

//UltrasonicSensor* above;
//UltrasonicSensor* below;
RemoteControl* rc;
//FilterOnePole lpf( LOWPASS, 5);


void plot(int Data1, int Data2, int Data3, int Data4=0, int Data5=0, int Data6=0, int Data7=0, int Data8=0)
{
  Serial.print(Data1); 
  Serial.print(" ");
  Serial.print(Data2); 
  Serial.print(" ");
  Serial.print(Data3); 
  Serial.print(" ");
  Serial.print(Data4); 
  Serial.print(" ");
  Serial.print(Data5); 
  Serial.print(" ");
  Serial.print(Data6); 
  Serial.print(" ");
  Serial.print(Data7); 
  Serial.print(" ");
  Serial.println(Data8); 
}

void setup() {
  Serial.begin(115200); // for debug
  SetRCInterrupts(); // for reading rc inputs / pinchanginterrupt

  // sensors to monitor distance above & below craft.
  //  above = new UltrasonicSensor(PIN_USENSOR_ABOVE_TRIG, PIN_USENSOR_ABOVE_ECHO);
//  below = new UltrasonicSensor(PIN_USENSOR_BELOW_TRIG, PIN_USENSOR_BELOW_ECHO);

  rc = new RemoteControl();
  rc->addChannel(new Channel(Channel::ELEVATOR, PIN_CH1_IN, PIN_CH1_OUT, 1150, 1850));
  rc->addChannel(new Channel(Channel::AILERON, PIN_CH2_IN, PIN_CH2_OUT, 1150, 1850));
  rc->addChannel(new Channel(Channel::THROTTLE, PIN_CH3_IN, PIN_CH3_OUT, 1100, 1820));
  rc->addChannel(new Channel(Channel::RUDDER, PIN_CH4_IN, PIN_CH4_OUT, 1150, 1850));
//#ifdef PIN_CH5
//  rc->addChannel(new ChannelOut(PIN_CH5, Channel::SWITCH_A, 990, 2000));
//#endif
//#ifdef PIN_CH6
//  rc->addChannel(new ChannelOut(PIN_CH6, Channel::SWITCH_B, 990, 2000));
//#endif

  // setup PID
//  floorInput = 0;
//  floorPID.SetOutputLimits(0, 100);
//  floorPID.SetMode(AUTOMATIC);
}

void loop() {
//  Serial.println("loop");
//  below->loop(); // loop sensor
  
  // listen to sensors
  // do PID for floor sensor then calculate throttle position
//  int r = below->read();
//  floorInput = r;
//  floorPID.Compute();

//  float p = floorOutput / 100;
//  if(r < 0) p = 0.5; // if there was an error with the sensor then send the throttle into hover
//  Channel* ch3 = rc->getChannel(Channel::THROTTLE);
//  ch3->setPerc(p); // set the percentage of throttle
////  Serial.print("Floor Sensor: ");
////  Serial.print((String)floorInput);
////  Serial.print(" -> ");
////  Serial.println(p);

  rc->loop(); // loop controls
  

//    //Add your repeated code here
//  int flag;
//  if(flag=getChannelsReceiveInfo()) //Here you read the RC flag contains all the channels that have a response
//                                    // see duane's excellent articles on how this works
//  {
//    plot(RC_Channel_Value[0],RC_Channel_Value[1],RC_Channel_Value[2],RC_Channel_Value[3],RC_Channel_Value[4],RC_Channel_Value[5]);
//  }
//  delay(50); //don't flood your serial monitor
}
