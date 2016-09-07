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
const int ESC::PWM_MAX = 180;
const int ESC::PWM_HIGH = 160;
const int ESC::PWM_LOW = 50;
const int ESC::PWM_NOSPIN = 20; // has to be somewhere between esc config off (stop) and low (slow spin) -jkr
const int ESC::PWM_OFF = 0;

ESC::ESC(int id) {
  this->_id = id;
  this->_pin = PIN_ESC_MOTOR[id];

//  this->_servo = new SoftwareServo();
  this->_servo.attach(this->_pin);

  if(false) { // debug
    Serial.print(F("ESC construct pin "));
    Serial.println(this->_pin);
  }
}

/**
 * `bypass` was introduced so that the esc's could arm before armed is set to true -jkr
 */
void ESC::setPWM(int pwm, bool bypass = false) {
  if(armed == false && this->_initialized == true && this->getPWM() != 0) {
    Serial.print(F("OFF"));
    Serial.print(F(" "));
    Serial.println(armed);
    this->_pwm = ESC::PWM_OFF;
    this->_servo.write(ESC::PWM_OFF); // running into a non-init conflict here -jkr

    return;
  }
  
  if(this->_pwm != pwm) { // only send a pwm change if the pwm has actually changed -jkr
    if(armed == true || bypass == true) {
      // leave restriction up to the 
//      if(armed == true && pwm < ESC::PWM_LOW) pwm = ESC::PWM_LOW; // only restirct PWM when armed -jkr
      
      this->_pwm = pwm;
      this->_servo.write(pwm);
  
      if(true) { // debug
        Serial.print(this->_id);
        Serial.print(" ");
        Serial.print(this->_pin);
        Serial.print("\t");
        Serial.println(pwm);
      }
    }
  }
  
}

void ESC::setPWMPerc(double p) {
    int v = p * ESC::PWM_MAX;
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
    this->setPWM(ESC::PWM_MAX, true);
    delay(ESC::INIT_MOTOR_INTERMISSION);
    this->setPWM(ESC::PWM_NOSPIN, true);
    delay(ESC::INIT_MOTOR_INTERMISSION);
    
    this->_initialized = true;

    Serial.print(F("ESC "));
    Serial.print(this->_id);
    Serial.println(F(" initialized"));
  }

  this->_servo.refresh();
}

