/**
 * may run into issues when drone is left sitting for some time because PID is still calculating. Worst case, have PID only run when armed == true -jkr
 */

#include <PID_v1.h>

#define PID_OUTPUT_MAX 255

class StabilityController {
  protected:
    RemoteControl* _rc;
    ROSController* _ros;
    IMUController* _imu;
    PID* _PIDPitch;
    PID* _PIDRoll;
    PID* _PIDPitchOpp;
    PID* _PIDRollOpp;
    float _r_yaw;
    double _pid_pitch_input, _pid_pitch_output, _pid_pitch_setpoint;
    double _pid_roll_input, _pid_roll_output, _pid_roll_setpoint;
    double _pid_pitchopp_input, _pid_pitchopp_output, _pid_pitchopp_setpoint;
    double _pid_rollopp_input, _pid_rollopp_output, _pid_rollopp_setpoint;
  public:
    StabilityController(RemoteControl* rc, ROSController* ros, IMUController *imu);
    void loop();
    const int PITCH_MAX = 25;
    const int ROLL_MAX = 25;
    const int YAW_MAX = 5;
    float* getOutput();

    static const float PID_KP;
    static const float PID_KI;
    static const float PID_KD;
    static const float PID_MULT;
    static const float GYRO_MULT_PITCH;
    static const float GYRO_MULT_ROLL;
    static const float GYRO_MULT_YAW;
};

    const float StabilityController::PID_KP =            0.0; // proportional
    const float StabilityController::PID_KI =            0.0; // integral
    const float StabilityController::PID_KD =            0.00; // derivative
    const float StabilityController::PID_MULT =          0.50; // influence of pid on output
    const float StabilityController::GYRO_MULT_PITCH =   0.00; // influence of gyro on output
    const float StabilityController::GYRO_MULT_ROLL =    0.00;
    const float StabilityController::GYRO_MULT_YAW =     0.00;

StabilityController::StabilityController(RemoteControl* rc, ROSController* ros, IMUController *imu) {
  this->_rc = rc;
  this->_ros = ros;
  this->_imu = imu;
  
  this->_PIDPitch = new PID(&this->_pid_pitch_input, 
                            &this->_pid_pitch_output, 
                            &this->_pid_pitch_setpoint, 
                            StabilityController::PID_KP, 
                            StabilityController::PID_KI, 
                            StabilityController::PID_KD, 
                            DIRECT
                          );
  this->_PIDRoll = new PID(&this->_pid_roll_input, 
                             &this->_pid_roll_output, 
                             &this->_pid_roll_setpoint, 
                             StabilityController::PID_KP, 
                             StabilityController::PID_KI, 
                             StabilityController::PID_KD, 
                             DIRECT
                           );
  this->_PIDPitchOpp = new PID(&this->_pid_pitchopp_input, 
                            &this->_pid_pitchopp_output, 
                            &this->_pid_pitchopp_setpoint, 
                            StabilityController::PID_KP, 
                            StabilityController::PID_KI, 
                            StabilityController::PID_KD, 
                            DIRECT
                          );
  this->_PIDRollOpp = new PID(&this->_pid_rollopp_input, 
                             &this->_pid_rollopp_output, 
                             &this->_pid_rollopp_setpoint, 
                             StabilityController::PID_KP, 
                             StabilityController::PID_KI, 
                             StabilityController::PID_KD, 
                             DIRECT
                           );

  this->_PIDPitch->SetMode(AUTOMATIC);
  this->_PIDRoll->SetMode(AUTOMATIC);
  this->_PIDPitchOpp->SetMode(AUTOMATIC);
  this->_PIDRollOpp->SetMode(AUTOMATIC);
}

/**
 * PRY output as a percentage, range -1 to 1 for each -jkr
 * eg. -1 pitch means rear motors are high and vice-versa
 * values are allowed to momentarily jump above 1 and below 0
 */
float* StabilityController::getOutput() {
   /**
   * get gyro output
   */

  float gyro_x, gyro_y, gyro_z;
  float* o = this->_imu->getGyro();
  gyro_x = o[0];
  gyro_y = o[1];
  gyro_z = o[2];
  delete [] o;

  if(false) {
    Serial.print(F("Gyro:\t"));
    Serial.print(gyro_x);
    Serial.print(F("\t"));
    Serial.print(gyro_y);
    Serial.print(F("\t"));
    Serial.println(gyro_z);
  }

  /**
   * get PID accelerometer
   */

  // change pid values to 0:1
  float p = (this->_pid_pitch_output / PID_OUTPUT_MAX) * StabilityController::PID_MULT;
  float r = (this->_pid_roll_output / PID_OUTPUT_MAX) * StabilityController::PID_MULT;

  // reduce by opposing PID's to make pids -1:1 -jkr
  p -= (this->_pid_pitchopp_output / PID_OUTPUT_MAX) * StabilityController::PID_MULT;
  r -= (this->_pid_rollopp_output / PID_OUTPUT_MAX) * StabilityController::PID_MULT;
  
  float y = this->_r_yaw / YAW_MAX;

  /**
   * adjust pr amounts based on which one is needed more 
   * high pitch is positive, high roll is negative -jkr
   */
  float pa = this->_pid_pitch_input / _pid_roll_input;
  float ra = this->_pid_roll_input / _pid_pitch_input;

  // factor in the results into the p and r values
  if(abs(pa) > abs(ra)) r = r / abs(pa);
  else p = p / abs(ra);
  
  if(false) {
    Serial.print(pa);
    Serial.print(F(" "));
    Serial.print(ra);
    Serial.print(F("\t\t"));
    Serial.print(r / abs(pa));
    Serial.print(F(" "));
    Serial.print(p / abs(ra));
    Serial.print(F("\t\t"));
    Serial.print(p);
    Serial.print(F(" "));
    Serial.println(r);
  }

  float gyro_x_comp = gyro_x * StabilityController::GYRO_MULT_PITCH;
  float gyro_y_comp = gyro_y * StabilityController::GYRO_MULT_ROLL;
  float gyro_z_comp = gyro_z * StabilityController::GYRO_MULT_YAW;

  // combine gyro values, attitude values, and pid values -jkr

  p -= gyro_x_comp;
  r -= gyro_y_comp;
  y -= gyro_z_comp;

  if(false) {
    Serial.print(p);
    Serial.print(F("\t"));
    Serial.print(r);
    Serial.print(F("\t"));
    Serial.println(y); // if gyro is off, this value will always be 0 -jkr
  }
  
  return new float[3] {p, r, y};
}

void StabilityController::loop() {

  /**
   * get desired drone position
   */
  
  float* o;
  float r_pitch = 0, r_roll = 0, r_yaw = 0;

  if(this->_ros->isOn()) {
    o = this->_ros->getAttitude();
    r_pitch = o[0];
    r_roll =  o[1];
    r_yaw =   o[2];
    delete [] o;
  }
  
  // rc takes precedence -jkr
  if(this->_rc->isOn()) {
    o = this->_rc->getAttitude();
    r_pitch = o[0];
    r_roll =  o[1];
    r_yaw =   o[2];
    delete [] o;
  }

  if(false) { // debug
    Serial.print("RC Percs: ");
    Serial.print(r_pitch);
    Serial.print(" ");
    Serial.print(r_roll);
    Serial.print(" ");
    Serial.println(r_yaw);
  }

  
  /**
   * get actual drone position
   */
  
  o = this->_imu->getAttitude();
  float imu_pitch, imu_roll, imu_yaw;
  imu_pitch = o[0];
  imu_roll  = o[1];
  imu_yaw   = o[2];
  delete [] o;

  if(false) {
    Serial.print(F("imu: "));
    Serial.print(imu_pitch);
    Serial.print(F(" "));
    Serial.print(imu_roll);
    Serial.print(F(" "));
    Serial.println(imu_yaw);
  }

  /**
   * compare actual position vs desired position and calculate PID
   */

  this->_pid_pitch_input = imu_pitch;
  this->_pid_roll_input = imu_roll;
  this->_pid_pitchopp_input = r_pitch;
  this->_pid_rollopp_input = r_roll;

  this->_pid_pitch_setpoint = r_pitch;
  this->_pid_roll_setpoint = r_roll;
  this->_pid_pitchopp_setpoint = imu_pitch;
  this->_pid_rollopp_setpoint = imu_roll;

  this->_r_yaw = r_yaw;

  // might need to change pid settings when we are close to the goal -jkr
  this->_PIDPitch->Compute();
  this->_PIDRoll->Compute();
  this->_PIDPitchOpp->Compute();
  this->_PIDRollOpp->Compute();

  if(false) {// && this->_rc->isOn()) {
    Serial.print(this->_pid_pitch_input);
    Serial.print(F(" "));
    Serial.print(this->_pid_pitch_setpoint);
    Serial.print(F(" "));
    Serial.print(this->_pid_pitch_output);
    Serial.print(F("\t\t"));
    Serial.print(this->_pid_pitchopp_input);
    Serial.print(F(" "));
    Serial.print(this->_pid_pitchopp_setpoint);
    Serial.print(F(" "));
    Serial.println(this->_pid_pitchopp_output);
  }

  // check the outputs only
  if(false) {
    Serial.print(this->_pid_pitch_output);
    Serial.print(F(" "));
    Serial.println(this->_pid_roll_output);
  }
}

