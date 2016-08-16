class ESC {
  protected:
    int _id;
    int _pin;
    int _pwm;
    bool _initialized = false;
    const int TIME_INIT_MILLIS = 1000;
    SoftwareServo _servo;
    int getPWM() {
      return this->_pwm;
    }
  public:
    ESC(int);
    void loop();
    int getId() {
      return this->_id;
    }
    void setPWMPerc(double p);
    void setPWM(int);
};

ESC::ESC(int id) {
  this->_id = id;
  this->_pin = PIN_ESC_MOTOR[id];

//  this->_servo = new SoftwareServo();
  this->_servo.attach(this->_pin);

  if(true) { // debug
    Serial.print("ESC construct pin ");
    Serial.println(this->_pin);
  }
}

void ESC::setPWM(int pwm) {
  if(this->_pwm != pwm && this->_initialized == true) {
    this->_pwm = pwm;
    this->_servo.write(pwm);

    if(true) {
      Serial.print(this->_id);
      Serial.print(" ");
      Serial.print(this->_pin);
      Serial.print(" ");
      Serial.println(pwm);
    }
  }
}

void ESC::setPWMPerc(double p) {
    int v = p * Channel::PWM_MAX;
//    if(v < Channel::PWM_LOW) v = Channel::PWM_LOW;
    this->setPWM(v);
}

void ESC::loop() {

  // attempt init
  if(millis() > TIME_INIT_MILLIS && this->_initialized == false) {
    this->_initialized = true;
    
//    this->_servo.write(Channel::PWM_MAX);
    this->setPWM(Channel::PWM_MAX);
    delay(5);
//    this->_servo.write(Channel::PWM_OFF);
    this->setPWM(Channel::PWM_NOSPIN);
    delay(5);

    Serial.print("ESC ");
    Serial.print(this->_id);
    Serial.println(" initialized");
  }

  this->_servo.refresh();
}

