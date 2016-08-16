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
    void setValue(int v) {
      this->_value = v;
    }
    int getValue() {
      return this->_value;
    }
    double getPerc() {
      double p = map(this->_value, this->_low, this->_high, 0, 1000);
      p /= 1000;
      if(p > 1) p = 1;
      if(p < 0) p = 0;
      return p;
    }
    String getType() {
      return _type;
    }
    
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
const int Channel::PWM_NOSPIN = 20;
const int Channel::PWM_OFF = 0;

Channel::Channel(String type, int inPin, int low, int high) {
  this->_type = type;
  this->_inPin = inPin;
  this->_high = high;
  this->_low = low;

  this->setValue(0);

//  if(this->_inPin != NULL) pinMode(this->_inPin, INPUT); // input is controlled included library
}

void Channel::loop() {
// INPUT
//  Serial.println("Channel Loop");
  
  if(this->_inPin != NULL) {
    for(int i = 0; i < NUM_RC_CHANNELS; ++i) {
      if(RC_Channel_Pin[i] == this->_inPin) {
////        if(this->getType() == "rudder") {
//          Serial.print("channel input: ");
//          Serial.print(this->_type);
//          Serial.print(": ");
//          Serial.print(RC_Channel_Value[i]);
//          Serial.println();
////        }
        this->setValue(RC_Channel_Value[i]);
      }
    }
  }
}

//void Channel::setPWM(int pwm) {
//  this->_pwm = pwm;
//}

//void Channel::setPerc(float perc) {
//  int pwm = this->_high - this->_low;
//  pwm *= perc;
//  pwm += this->_low;
//  this->setPWM(pwm);
//}

//void Channel::assumeDefaultPWM() {
//  if(this->_low > 0 && this->_high > 0) {
//    // applies to all channel types
//    this->_pwm = (this->_high - this->_low) / 2 + this->_low; // should be the median between low and high
//    
//    if(this->_type == Channel::THROTTLE) // override for throttle
//      this->_pwm = this->_low;
//  }
//}
