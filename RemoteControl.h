class RemoteControl {
  public:
    RemoteControl();
    Channel* getChannel(String type);
    void move(int vector, int speed);
    int addChannel(Channel* ch);
    void loop();
  private:
    Channel* _channels[6];
    int _channels_length;
};

RemoteControl::RemoteControl() {
  this->_channels_length = 0;
}

/**
 * return channel index
 */
int RemoteControl::addChannel(Channel* ch) {
  this->_channels[this->_channels_length++] = ch;

  return this->_channels_length;
}

void RemoteControl::loop() {
  if(getChannelsReceiveInfo()) {
    Serial.println();
    for(int i = 0; i < this->_channels_length; ++i) {
      Channel* o = this->_channels[i];
      o->loop();
      Serial.print(o->getValue());
      Serial.print(" ");
    }
  }
}

Channel* RemoteControl::getChannel(String type) {
  for(int i = 0; i < this->_channels_length; ++i) {
    Channel* o = this->_channels[i];
    if(o->getType() == type) return o;
  }

  return NULL;
}

