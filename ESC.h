class ESC {
  protected:
    int _id;
    int _pin;
    int _pwm;
    bool _initialized = false;
    const int TIME_INIT_MILLIS = 1000;
    SoftwareServo _servo;
  public:
    ESC(int);
    void loop();
    int getId() {
      return this->_id;
    }
    bool getInitialized() {
      return this->_initialized;
    }
    void setPWMPerc(double p);
    void setPWM(int);
    int getPWM() {
      return this->_pwm;
    }
};

ESC::ESC(int id) {
  this->_id = id;
  this->_pin = PIN_ESC_MOTOR[id];

//  this->_servo = new SoftwareServo();
  this->_servo.attach(this->_pin);

  if(false) { // debug
    Serial.print("ESC construct pin ");
    Serial.println(this->_pin);
  }
}

void ESC::setPWM(int pwm) {
  if(armed == true && this->_pwm != pwm) {
    if(pwm < Channel::PWM_LOW) pwm = Channel::PWM_LOW;
    
    this->_pwm = pwm;
    this->_servo.write(pwm);

    if(false) { // debug
      Serial.print(this->_id);
      Serial.print(" ");
      Serial.print(this->_pin);
      Serial.print(" ");
      Serial.println(pwm);
    }
  } else if(armed == false) {
    this->_servo.write(Channel::PWM_OFF);
  }
}

void ESC::setPWMPerc(double p) {
    int v = p * Channel::PWM_MAX;
    this->setPWM(v);

    if(false) {
      Serial.print(p);
      Serial.print(" ");
      Serial.println(v);
    }
}

void ESC::loop() {

  // attempt init
  if(millis() > TIME_INIT_MILLIS && this->_initialized == false) {
    this->_initialized = true;
    
    this->_servo.write(Channel::PWM_MAX);
//    this->setPWM(Channel::PWM_MAX); // not armed yet, have to bypass init -jkr
    delay(5);
    this->_servo.write(Channel::PWM_NOSPIN); // needs to be nospin (very low) not off (0), off makes the esc ANGRY >: -jkr
//    this->setPWM(Channel::PWM_NOSPIN);; // not armed yet, have to bypass init -jkr
    delay(5);

    Serial.print("ESC ");
    Serial.print(this->_id);
    Serial.println(" initialized");
  }

  this->_servo.refresh();
}

