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
    int _outPin;
    int _high;
    int _low;
  public:
    void loop();
    int getValue() {
      return this->_value;
    }
    String getType() {
      return _type;
    }
    
    static const String ELEVATOR;
    static const String AILERON;
    static const String THROTTLE;
    static const String RUDDER;
    static const String RANGE_A;
    static const String RANGE_B;

    Channel(String type, int inPin, int outPin, int high, int low);
};

const String Channel::ELEVATOR = "elevator";
const String Channel::AILERON  = "aileron";
const String Channel::THROTTLE = "throttle";
const String Channel::RUDDER   = "rudder";
const String Channel::RANGE_A  = "range_a";
const String Channel::RANGE_B  = "range_b";

Channel::Channel(String type, int inPin, int outPin, int high, int low) {
  this->_type = type;
  this->_inPin = inPin;
  this->_outPin = outPin;
  this->_high = high;
  this->_low = low;

  // using RCLib + PinChangeInt libs now - jkr
//  if(this->_inPin != NULL) pinMode(this->_inPin, INPUT);
  if(this->_outPin != NULL) pinMode(this->_outPin, OUTPUT);
}

void Channel::loop() {
  // INPUT
  if(this->_inPin != NULL) {
    // using RCLib + PinChangeInt libs now - jkr
//    this->_pulseIn = pulseIn(this->_inPin, HIGH, 25000);
//  
//    if(this->_pulseIn > 0 && true) {
//      String str = this->getType();
//      str += " in: ";
//      str += this->getPulseIn();
//      Serial.println(str);
//    }

    int flag;
    if(flag = getChannelsReceiveInfo()) {
      for(int i = 0; i < NUM_RC_CHANNELS; ++i) {
        if(RC_Channel_Pin[i] == this->_inPin) 
          this->_value = RC_Channel_Value[i];
      }
    }
  }

  if(this->_outPin != NULL) {
    
  }
    
  // OUTPUT
//  float mv = map(this->_pwm, this->_low, this->_high, 0, 255);
//  analogWrite(this->_pin, mv);
//
//  if(false) {
//    Serial.print(this->_type);
//    Serial.print(" out: ");
//    Serial.println(mv);
//  }
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
