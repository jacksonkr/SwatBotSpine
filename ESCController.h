class ESCController {
  protected:
    /**
     * percentage of how much the throttle influences motor speed
     */
    const double INFLUENCE_EMERGENCY   = 0.1; // 10%
    const double INFLUENCE_PID         = 0.2; // 20%
    const double INFLUENCE_THROTTLE    = 0.7; // 70%

    RemoteControl* _rc;
    ROSController* _ros;
    StabilityController* _sc;
    ESC* _escFamily[NUM_ESC_MOTORS];
    double _starting_motor_angle = 90;
    double _motor_angle_spread;
    double getMotorArmAngleById(int id) {
      double v = this->_starting_motor_angle - id * this->_motor_angle_spread;
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

  if(false) { // debug
    for(int i = 0; i < NUM_ESC_MOTORS; ++i) {
      double v = this->getMotorArmAngleById(i);
      Serial.print("Motor ");
      Serial.print(i);
      Serial.print(" angle: ");
      Serial.println(v);
    }
  }
}

void ESCController::loop() {
  /**
   * get new values from sc
   */

  double pitch, roll, yaw;
  double* attitude = this->_sc->getPIDOutput();
  pitch = attitude[0];
  roll = attitude[1];
  yaw = 0; // attitude[2];
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

  double thr_p = 0;

  if(this->_rc->isOn() == true)
    thr_p = this->_rc->getChannel(Channel::THROTTLE)->getPerc();

  if(this->_ros->isOn() == true)
    thr_p = this->_ros->getThrottlePerc();

  if(armed == false) thr_p = ESC::PWM_OFF;

#ifdef SERIAL_IN
  char rin = Serial.read();
  double rin_val = -1;
  char inStr = rin;
  if(rin > -1) {
    Serial.print("serial in: ");
    Serial.print(inStr);

    Serial.println(F("!! changed inStr from String to char, this may break things -jkr"));
    if ((int)inStr >= 0 && (int)inStr <= 9) {
      rin_val = (int)inStr * 20;
      if(rin_val > 0) {
        Serial.print(" ");
        Serial.println(rin_val);
      }
    }
  }
#endif
  
  for(int i = 0; i < NUM_ESC_MOTORS; ++i) {
    ESC* o = this->_escFamily[i];

    int arm_angle = this->getMotorArmAngleById(i);
    double rad = arm_angle / 360 * PI;
    double pitch_mult, roll_mult, yaw_mult;

    //todo: make this dynamic to fit any style uas
    // amount of effect each motor has based on pitch vs roll
    int with_rc_values = 1;
    int against_rc_values = -1;
    if(NUM_ESC_MOTORS == 4) {
      // STATIC FOR X QUAD
      switch(i){
        case 0:
          pitch_mult  = with_rc_values;
          roll_mult   = with_rc_values;
          yaw_mult    = with_rc_values;
          break;
        case 1:
          pitch_mult  = with_rc_values;
          roll_mult   = against_rc_values;
          yaw_mult    = against_rc_values;
        break;
        case 2:
          pitch_mult  = against_rc_values;
          roll_mult   = against_rc_values;
          yaw_mult    = with_rc_values;
        break;
        case 3:
          pitch_mult  = against_rc_values;
          roll_mult   = with_rc_values;
          yaw_mult    = against_rc_values;
        break;
      }
    }
    
    double p =    pitch * pitch_mult;
    double r =    roll * roll_mult;
    double y =    yaw * yaw_mult;
    double t1 =   thr_p * INFLUENCE_THROTTLE;
    double t2 =   thr_p * INFLUENCE_PID * ((p + r + y) / 3);
    double t =    t1 + t2;

    if(false && this->_rc->isOn()) {
      Serial.print(i);
      Serial.print(" ");
      Serial.print(thr_p);
      Serial.print(" ");
      Serial.print(p);
      Serial.print("\t");
      Serial.print(r);
      Serial.print("\t");
      Serial.print(y);
      Serial.print("\t");
      Serial.println(t);
    }

    if(false && i == 0) { // for a specific arm
      Serial.print(thr_p);
      Serial.print("\t");
      Serial.print(p);
      Serial.print("\t");
      Serial.print(r);
      Serial.print("\t");
      Serial.println(t);
      
      t = thr_p;// DEBUG - send out throttle value only (no pid compensation / no stabilization) -jkr
    }

#ifdef SERIAL_IN
    if(rin_val > -1)
      o->setPWM(rin_val);
#endif

    if(this->_rc->isOn() == true || this->_ros->isOn() == true)
      o->setPWMPerc(t);
    else if(armed == false) {
      o->setPWM(ESC::PWM_OFF);
    }

    o->loop();

  }
}

