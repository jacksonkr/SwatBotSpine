class ESCController {
  protected:
    RemoteControl* _rc;
    ROSController* _ros;
    StabilityController* _sc;
    ESC* _escFamily[NUM_ESC_MOTORS];
    int _starting_motor_angle = 90;
    float _motor_angle_spread;
    float getMotorArmAngleById(int id) {
      float v = this->_starting_motor_angle - id * this->_motor_angle_spread;
      if(v < 0) v += 360;
      return v;
    }
  public:
    ESCController(RemoteControl* rc, ROSController* ros, StabilityController* sc);
    void loop();
    bool getInitialized() {
       /**
       * see if all motors were initialized
       */
      bool inited = true;
      for(int i = 0; i < NUM_ESC_MOTORS; ++i) {
        ESC* o = this->_escFamily[i];
        
        if(o->getInitialized() != true)
          inited = false;
        }
      
      return inited;
    }
};

ESCController::ESCController(RemoteControl* rc, ROSController* ros, StabilityController* sc) {
  this->_rc = rc;
  this->_ros = ros;
  this->_sc = sc;

  for(int i = 0; i < NUM_ESC_MOTORS; ++i) {
    this->_escFamily[i] = new ESC(i);
  }

  this->_motor_angle_spread = 360 / NUM_ESC_MOTORS;
  
  if(UAV_TYPE == TYPE_X) {
    this->_starting_motor_angle += this->_motor_angle_spread / 2; // motor 1 angle pretty much
  }

  Serial.println("Connect Power");

  // todo: get arm distances to make p r y calculations work for any number of arms
//    for(int i = 0; i < NUM_ESC_MOTORS; ++i) {
//      float dist = this->getMotorArmAngleById(i);
//    }

// todo: goes with the above somehow -jkr
//    int arm_angle = this->getMotorArmAngleById(i);
//    float rad = (float)arm_angle / 180 * PI;
//    float pitch_mult = cos(rad);
//    float roll_mult = sin(rad);
//    float yaw_mult = 1;
//
//    if(false) {
//      Serial.print(i);
//      Serial.print(F("\t"));
//      Serial.print(rad);
//      Serial.print(F("\t"));
//      Serial.print(arm_angle);
//      Serial.print(F("\t"));
//      Serial.print(pitch_mult);
//      Serial.print(F("\t"));
//      Serial.println(roll_mult);
//    }
}

void ESCController::loop() {
  /**
   * get new values from sc
   */

  float pitch, roll, yaw;
  float* attitude = this->_sc->getOutput();
  pitch = attitude[0];
  roll = attitude[1];
  yaw = attitude[2];
  delete [] attitude;

  if(false) {
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.println(yaw);
  }

  /**
   * assign to motors
   */

  float thr_p = 0;

  if(this->_rc->isOn() == true)
    thr_p = this->_rc->getChannel(Channel::THROTTLE)->getPerc();

  if(this->_ros->isOn() == true)
    thr_p = this->_ros->getThrottlePerc();

  if(armed == false) thr_p = ESC::PWM_OFF;
  
  for(int i = 0; i < NUM_ESC_MOTORS; ++i) {
    ESC* o = this->_escFamily[i];

    //todo: this is static for time's sake. Dynamic portion yet to be finished (notes everywhere) -jkr
    // amount of effect each motor has based on pitch vs roll
    float pitch_mult, roll_mult, yaw_mult;
    int with_values = 1;
    int against_values = -1;
    if(NUM_ESC_MOTORS == 4) {
      // STATIC FOR X QUAD w front at -180 degress
      switch(i){
        case 0:
          pitch_mult  = with_values;
          roll_mult   = with_values;
          yaw_mult    = with_values;
          break;
        case 1:
          pitch_mult  = with_values;
          roll_mult   = against_values;
          yaw_mult    = against_values;
        break;
        case 2:
          pitch_mult  = against_values;
          roll_mult   = against_values;
          yaw_mult    = with_values;
        break;
        case 3:
          pitch_mult  = against_values;
          roll_mult   = with_values;
          yaw_mult    = against_values;
        break;
      }
    }

    /**
     * This is the heart of the algo
     * shouldn't this be in the stabilty controller? yes -jkr1
     */
    
    double p =    pitch * pitch_mult;
    double r =    roll * roll_mult;
    double y =    yaw * yaw_mult;
    double t = thr_p + ((p + r + y) / 3); // totals 2.0 but this allows PID to take max throttle in moments when needed (until max buffer kicks in)
    if(t > 1) t = 1;

    // todo: I'm going to have to calculate the pwm for each motor BEFORE the loop, then add a max buffer if any pwms are above the max (bring the others dowm proportionately) -jkr

    if(false) { // "visual" debug -jkr
//      t = thr_p;
      switch(i) {
        case 0:
          Serial.println(F(" "));
          Serial.print(F("-----------"));
          Serial.print(F(" "));
          Serial.print(p);
          Serial.print(F(" "));
          Serial.print(r);
          Serial.print(F(" "));
          Serial.print(y);
          Serial.print(F(" "));
          Serial.println(F("------------"));
          Serial.println(F(" "));
        case 2:
          Serial.print(t);
          Serial.print(F("\t\t"));
        break;
        case 1:
          Serial.println(t);
          Serial.println(F(" "));
          Serial.print("\t");
          Serial.print(thr_p);
          Serial.println(F(" "));
          Serial.println(F(" "));
        break;
        case 3:
          Serial.println(t);
        break;
      }
    }

    if(this->_rc->isOn() == true || this->_ros->isOn() == true)
      o->setPWMPerc(t);
    else if(armed == false) {
      o->setPWM(ESC::PWM_OFF);
    }

    o->loop();
  }
//      delay(5);
}

