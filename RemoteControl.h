class RemoteControl {
  protected:
    Channel* _channels[6];
    int _channels_length;
  public:
    RemoteControl();
    Channel* getChannel(String type);
    void move(int vector, int speed);
    int addChannel(Channel* ch);
    void loop();
    void setup();
    double* getAttitude();
    bool isOn() {
      return getChannelsReceiveInfo();
    }
};

RemoteControl::RemoteControl() {
  this->_channels_length = 0;
}

double* RemoteControl::getAttitude() {
  // figure desired euler from RX
  double ele_p = this->getChannel(Channel::ELEVATOR)->getPerc();
  double ail_p = this->getChannel(Channel::AILERON)->getPerc();
  double rud_p = this->getChannel(Channel::RUDDER)->getPerc();

  if(false && this->isOn()) { // debug
    Serial.print(ele_p);
    Serial.print(" ");
    Serial.print(ail_p);
    Serial.print(" ");
    Serial.print(rud_p);
  //  Serial.print(" ");
  //  Serial.print(thr_p);
//    if(NUM_RC_CHANNELS >= 5) {
//      Serial.print(" ");
//      Serial.print(a1p);
//    }
//    if(NUM_RC_CHANNELS >= 6) {  
//      Serial.print(" ");
//      Serial.print(a2p);
//    }
    Serial.println();
  }

  double p, r, y;

  p = (ele_p - 0.5) * 2 * PITCH_MAX;
  r = (ail_p - 0.5) * 2 * ROLL_MAX;
  y = (rud_p - 0.5) * 2 * YAW_MAX;

  return new double[3]{p, r, y};
}

/**
 * return channel index
 */
int RemoteControl::addChannel(Channel* ch) {
//  ch->addRemoteControl(this);
  this->_channels[this->_channels_length++] = ch;

  return this->_channels_length;
}

Channel* RemoteControl::getChannel(String type) {
  for(int i = 0; i < this->_channels_length; ++i) {
    Channel* o = this->_channels[i];
    if(o->getType() == type) return o;
  }

  return NULL;
}

void RemoteControl::loop() {
//  Serial.println("RC Loop");

  if(this->isOn()) {
    for(int i = 0; i < this->_channels_length; ++i) {
      Channel* o = this->_channels[i];
      o->loop();

      if(false) { // rc debug
        Serial.print(o->getValue());
        Serial.print(" ");

        if(i >= this->_channels_length - 1) Serial.println();
      }
    }
  }
    
//  Serial.println();
}

