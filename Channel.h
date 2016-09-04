#pragma once
#include <WString.h>
#include <RCLib.h>

class Channel {
  private:
    int _chType;
    int _value;
  protected:
    String _type;
    int _inPin;
    int _high;
    int _low;
  public:
    void loop();
    void setValue(int);
    int getValue();
    double getPerc();
    String getType();
    
    static const String ELEVATOR;
    static const String AILERON;
    static const String THROTTLE;
    static const String RUDDER;
    static const String AUX1;
    static const String AUX2;

    static const int PWM_MAX;
    static const int PWM_HIGH;
    static const int PWM_LOW;
    static const int PWM_NOSPIN;
    static const int PWM_OFF;

    Channel(String type, int inPin, int low, int high);
};

const String Channel::ELEVATOR = "elevator";
const String Channel::AILERON  = "aileron";
const String Channel::THROTTLE = "throttle";
const String Channel::RUDDER   = "rudder";
const String Channel::AUX1     = "aux1";
const String Channel::AUX2     = "aux2";

const int Channel::PWM_MAX = 180;
const int Channel::PWM_HIGH = 160;
const int Channel::PWM_LOW = 40;
const int Channel::PWM_NOSPIN = 20;
const int Channel::PWM_OFF = 0;

Channel::Channel(String type, int inPin, int low, int high) {
  this->_type = type;
  this->_inPin = inPin;
  this->_high = high;
  this->_low = low;

//  this->setValue(0);

//  if(this->_inPin != NULL) pinMode(this->_inPin, INPUT); // input is controlled included library
}
void Channel::setValue(int v) {
//  if(armed == true && v < Channel::PWM_NOSPIN) v = Channel::PWM_NOSPIN;
  this->_value = v;
}
int Channel::getValue() {
  return this->_value;
}
double Channel::getPerc() {
  double p = map(this->getValue(), this->_low, this->_high, 0, 1000);
  p /= 1000;
  if(p > 1) p = 1;
  if(p < 0) p = 0;
  return p;
}
String Channel::getType() {
  return this->_type;
}

void Channel::loop() {
//  Serial.println("Channel Loop");
  if(this->_inPin != NULL) {
    for(int i = 0; i < NUM_RC_CHANNELS; ++i) {
      if(RC_Channel_Pin[i] == this->_inPin) {
        if(false && this->getType() == "throttle") { // debug
          Serial.print(F("channel input: "));
          Serial.print(this->_type);
          Serial.print(F(": "));
          Serial.print(RC_Channel_Value[i]);
          Serial.println();
        }
        this->setValue(RC_Channel_Value[i]);
      }
    }
  }
}
