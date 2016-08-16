class ESCController {
  protected:
    /**
     * percentage of how much the throttle influences motor speed
     */
    const double THROTTLE_INFLUENCE = 0.8;
    const double PID_INFLUENCE = 0.2;
  
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
    void setMode(String mode);
    void loop();
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

  double pitch, roll;
  double* attitude = this->_sc->getPIDOutput();
  pitch = attitude[0];
  roll = attitude[1];
  delete [] attitude;

  if(false) {
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
  }

  /**
   * assign to motors
   */

  double thr_p = 0;

  if(this->_rc->isOn() == true)
    thr_p = this->_rc->getChannel(Channel::THROTTLE)->getPerc();

  if(this->_ros->isOn() == true)
    thr_p = this->_ros->getThrottlePerc();

#ifdef SERIAL_IN
  char rin = Serial.read();
  double rin_val = -1;
  String inStr = String(rin);
  if(rin > -1) {
    Serial.print("serial in: ");
    Serial.print(inStr);
    
    if (inStr.toInt() >= 0 && inStr.toInt() <= 9) {
      rin_val = inStr.toInt() * 20;
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
    double pitch_mult, roll_mult;

    // STATIC FOR X QUAD
    switch(i){
      case 0:
        pitch_mult = 1;
        roll_mult = -1;
        break;
      case 1:
        pitch_mult = 1;
        roll_mult = 1;
      break;
      case 2:
        pitch_mult = -1;
        roll_mult = 1;
      break;
      case 3:
        pitch_mult = -1;
        roll_mult = -1;
      break;
    }
    
    double p = pitch * pitch_mult;
    double r = roll * roll_mult;
    double t1 = thr_p * THROTTLE_INFLUENCE;
    double t2 = thr_p * (p + r) * PID_INFLUENCE;
    double t = t1 + t2;
    t = thr_p;// manual override
//    if(t < 0) t = 0;

    if(false) {
      Serial.print(t1);
      Serial.print(" ");
      Serial.println(t2);
      Serial.print(" ");
      Serial.println(t);
    }

    if(false) {
      Serial.print(pitch);
      Serial.print(" ");
      Serial.print(pitch_mult);
      Serial.print(" ");
      Serial.print(thr_p);
      Serial.print(" ");
      Serial.print(p);
      Serial.print(" ");
      Serial.print(r);
      Serial.print(" ");
      Serial.println(t);
    }

#ifdef SERIAL_IN
    if(rin_val > -1)
      o->setPWM(rin_val);
#endif

    if(this->_rc->isOn() == true || this->_ros->isOn())
      o->setPWMPerc(t);

    o->loop();
  }
}

