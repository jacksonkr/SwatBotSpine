#pragma once
#include <WString.h>
#include <RCLib.h>

class Channel {
  private:
    int _chType;
    int _value;
  protected:
    const char* _type;
    int _inPin;
    int _high;
    int _low;
  public:
    void loop();
    void setValue(int);
    int getValue();
    double getPerc();
    const char* getType();
    
    static const char* ELEVATOR;
    static const char* AILERON;
    static const char* THROTTLE;
    static const char* RUDDER;
    static const char* AUX1;
    static const char* AUX2;

    Channel(const char* type, int inPin, int low, int high);
};

const char* Channel::ELEVATOR = "elevator";
const char* Channel::AILERON  = "aileron";
const char* Channel::THROTTLE = "throttle";
const char* Channel::RUDDER   = "rudder";
const char* Channel::AUX1     = "aux1";
const char* Channel::AUX2     = "aux2";

Channel::Channel(const char* type, int inPin, int low, int high) {
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
const char* Channel::getType() {
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
