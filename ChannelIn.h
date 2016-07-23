#include "Channel.h"

class ChannelIn : public Channel {
  private:
    int _pulseIn;
  public:
    ChannelIn(String type, int inPin, int high, int low, int outPin);
    void loop();
    int getPulseIn() {
      return this->_pulseIn;
    }
    int getOutPin() {
      return this->_pin;
    }
    int getPWM() {
      return this->_pwm;
    }
    void loop();
    void setPWM(int pwm);
    void setPerc(float perc);
    
};

ChannelIn::ChannelIn(String type, int inPin, int high, int low, int outPin) : Channel(type, inPin, outPin, high, low) {
  this->_type = type;
  this->_pin = inPin;

  pinMode(this->_pin, INPUT);
}

void ChannelIn::loop() {
  this->_pulseIn = pulseIn(this->_pin, HIGH, 25000);
  
  if(true) {
    String str = this->getType();
    str += " in: ";
    str += this->getPulseIn();
    Serial.println(str);
  }
}
