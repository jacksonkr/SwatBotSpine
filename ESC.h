class ESC {
  protected:
    int _id;
    int _pin;
    int _pwm;
    int _init_time = -1;
    const int TIME_INIT_MILLIS = 1000;
    SoftwareServo _servo;
    bool beyondInitPad() {
      return millis() - this->_init_time > 4000;
    }
  public:
    ESC(int);
    void loop();
    int getId() {
      return this->_id;
    }
    bool getInitialized() {
      return this->_init_time > 0;
    }
    void setPWMPerc(double);
    void setPWM(int, bool);
    int getPWM() {
      return this->_pwm;
    }

    static const int INIT_MOTOR_INTERMISSION;
    static const int PWM_MAX;
    static const int PWM_HIGH;
    static const int PWM_LOW;
    static const int PWM_NOSPIN;
    static const int PWM_OFF;
};

const int ESC::INIT_MOTOR_INTERMISSION = 20; // time before and after high low during intermission -jkr
const int ESC::PWM_MAX = 180; // max pwm but this sends a slow kill to the motors
const int ESC::PWM_HIGH = 160; // fastest documented pwm to spin motor
const int ESC::PWM_LOW = 50; // lowest documented pwm to spin motor
const int ESC::PWM_NOSPIN = 25; // has to be somewhere between esc config off (stop) and low (slow spin) -jkr
const int ESC::PWM_OFF = 0;

ESC::ESC(int id) {
  this->_id = id;
  this->_pin = PIN_ESC_MOTOR[id];

  this->_servo.attach(this->_pin); // this->_servo = new SoftwareServo();

  if(false) { // debug
    Serial.print(F("ESC construct pin "));
    Serial.println(this->_pin);
  }
}

/**
 * `bypass` was introduced so that the esc's could arm before armed is set to true -jkr
 */
void ESC::setPWM(int pwm, bool bypass = false) {
  if( armed == false
  && this->getPWM() != ESC::PWM_OFF 
  && this->_init_time > 0
  && this->beyondInitPad() == true 
  ) {
    Serial.print(F("OFF"));
    Serial.print(F(" "));
    Serial.println(armed);
    this->_pwm = ESC::PWM_OFF;
    this->_servo.write(ESC::PWM_OFF); // running into a non-init conflict here -jkr

    return;
  }

  if(pwm < ESC::PWM_LOW && this->beyondInitPad() == true) pwm = ESC::PWM_LOW; // keep the motors from slowing to a stop in flight
  if(pwm > ESC::PWM_HIGH) pwm = ESC::PWM_HIGH;
  
  if(this->_pwm != pwm) { // only send a pwm change if the pwm has actually changed -jkr
    if(armed == true || bypass == true) {
      this->_pwm = pwm;
      this->_servo.write(pwm);
  
      if(false && this->_id == 0) { // debug
        Serial.print(this->_id);
        Serial.print(F(" "));
        Serial.print(this->_pin);
        Serial.print("\t");
        Serial.println(pwm);
      }
    }
  }
}

void ESC::setPWMPerc(double p) {
    int v = 0; // = p * ESC::PWM_MAX;
    v = map(p * 100, 0, 100, ESC::PWM_LOW, ESC::PWM_HIGH); // use high, not max. max will shut off motors -jkr
    this->setPWM(v);

    if(false) {// && this->_id == 0) {
      Serial.print(this->_id);
      Serial.print(F("\t"));
      Serial.print(p);
      Serial.print(F(" "));
      Serial.println(v);
    }
}

void ESC::loop() {
  // attempt init
  if(millis() > TIME_INIT_MILLIS && this->_init_time < 0) { // first init
    
    this->setPWM(ESC::PWM_MAX, true);
    delay(ESC::INIT_MOTOR_INTERMISSION);
    this->setPWM(ESC::PWM_NOSPIN, true);
    delay(ESC::INIT_MOTOR_INTERMISSION);

    this->_init_time = millis();

    Serial.print(F("ESC "));
    Serial.print(this->_id);
    Serial.println(F(" initialized"));
  }

  this->_servo.refresh();
}

