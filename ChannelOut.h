#include "Channel.h"

class ChannelOut : public Channel {
  public:
    ChannelOut(int outPin, String type, int low, int high, int inPin);
    int getOutPin() {
      return this->_pin;
    }
    int getPWM() {
      return this->_pwm;
    }
    void loop();
    void setPWM(int pwm);
    void setPerc(float perc);
  private:
    int _pwm;
    void assumeDefaultPWM();
    int _low;
    int _high;
};

ChannelOut::ChannelOut(int outPin, String type, int low, int high, int inPin) : Channel(type, inPin, outPin, high, low) {
  this->_type = type;
  this->_pin = outPin;
  this->_low = low;
  this->_high = high;
  
  this->assumeDefaultPWM();

//  pinMode(this->_pin, OUTPUT); // not needed for analogWrite
}

void ChannelOut::loop() {
    float mv = map(this->_pwm, this->_low, this->_high, 0, 255);
    analogWrite(this->_pin, mv);

    if(false) {
      Serial.print(this->_type);
      Serial.print(" out: ");
      Serial.println(mv);
    }
}

void ChannelOut::setPWM(int pwm) {
  this->_pwm = pwm;
}

void ChannelOut::setPerc(float perc) {
  int pwm = this->_high - this->_low;
  pwm *= perc;
  pwm += this->_low;
  this->setPWM(pwm);
}

void ChannelOut::assumeDefaultPWM() {
  if(this->_low > 0 && this->_high > 0) {
    // applies to all channel types
    this->_pwm = (this->_high - this->_low) / 2 + this->_low; // should be the median between low and high
    
    if(this->_type == Channel::THROTTLE) // override for throttle
      this->_pwm = this->_low;
  }
}
