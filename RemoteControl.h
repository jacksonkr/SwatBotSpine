class RemoteControl {
  protected:
    Channel* _channels[NUM_RC_CHANNELS];
    int _channels_length;
    double _channel_receive_info_arr[10];
    bool _remote_is_on = false;
    void loopChannels();
  public:
    RemoteControl();
    Channel* getChannel(String type);
    void filterRemoteOnError();
    void move(int vector, int speed);
    int addChannel(Channel* ch);
    void loop();
    void setup();
    double* getAttitude();
    bool isOn() {
      return this->_remote_is_on;
    }
};

RemoteControl::RemoteControl() {
  this->_channels_length = 0;
  
  this->addChannel(new Channel(Channel::AILERON,   RC_Channel_Pin[0], 1150, 1850));
  this->addChannel(new Channel(Channel::ELEVATOR,  RC_Channel_Pin[1], 1150, 1850));
  this->addChannel(new Channel(Channel::THROTTLE,  RC_Channel_Pin[2], 1150, 1820));
  this->addChannel(new Channel(Channel::RUDDER,    RC_Channel_Pin[3], 1150, 1850));
  if(RC_Channel_Pin[5])
    this->addChannel(new Channel(Channel::AUX1, RC_Channel_Pin[4], 990, 2000));
  if(RC_Channel_Pin[6])
    this->addChannel(new Channel(Channel::AUX2, RC_Channel_Pin[5], 990, 2000));
}

double* RemoteControl::getAttitude() {
  // figure desired euler from RX
  double ele_p = this->getChannel(Channel::ELEVATOR)->getPerc();
  double ail_p = this->getChannel(Channel::AILERON)->getPerc();
  double rud_p = this->getChannel(Channel::RUDDER)->getPerc();

  if(false) { // debug
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

  if(false) { // debug
    Serial.print(p);
    Serial.print(" ");
    Serial.print(r);
    Serial.print(" ");
    Serial.println(y);
  }

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

    if(false) {
      Serial.print(type);
      Serial.print(" == ");
      Serial.println(o->getType());
    }
    
    if(o->getType() == type) return o;
  }

  return NULL;
}

// the remote always reports 0 when off but sometimes (rarely) reports 0 when on; this is a filter for that small error
void RemoteControl::filterRemoteOnError() {
  // keep track of of receiver activity to compensate for error; sometimes the rc reports 0 when it should report 1 -jkr
  int len = sizeof(this->_channel_receive_info_arr) / sizeof(*this->_channel_receive_info_arr);
  arrayShift(this->_channel_receive_info_arr, getChannelsReceiveInfo(), len);
  
  this->_remote_is_on = false;
  for(int i=0; i<len; ++i) {
    if(false) { // debug
      Serial.print(this->_channel_receive_info_arr[i]);
      Serial.print(" ");
      if(i >= len-1) Serial.println(" ");
    }
    if(this->_channel_receive_info_arr[i] > 0)
      this->_remote_is_on = true;
  }
}

void RemoteControl::loopChannels() {
  for(int i = 0; i < this->_channels_length; ++i) {
    Channel* o = this->_channels[i];
    o->loop();

    if(false) { // rc debug
      Serial.print(o->getValue());
      Serial.print(" ");
      Serial.println(millis());

      if(i >= this->_channels_length - 1) Serial.println();
    }
  }
}

void RemoteControl::loop() {
//  Serial.print("RC Loop");

  this->filterRemoteOnError();

  if(this->isOn()) 
    this->loopChannels();
}

