#include <PID_v1.h>

#define PID_OUTPUT_MAX 255;

class StabilityController {
  protected:
    RemoteControl* _rc;
    ROSController* _ros;
    IMUController* _imu;
    PID* _PIDPitchPos;
    PID* _PIDRollPos;
    PID* _PIDPitchNeg;
    PID* _PIDRollNeg;
    double _r_yaw;
    double _pid_pitchp_input, _pid_pitchp_output, _pid_pitchp_setpoint;
    double _pid_pitchn_input, _pid_pitchn_output, _pid_pitchn_setpoint;
    double _pid_rollp_input, _pid_rollp_output, _pid_rollp_setpoint;
    double _pid_rolln_input, _pid_rolln_output, _pid_rolln_setpoint;
  public:
    StabilityController(RemoteControl* rc, ROSController* ros, IMUController *imu);
    void loop();
    const double PITCH_MAX = 25;
    const double ROLL_MAX = 25;
    const double YAW_MAX = 5;
    double* getPRYOutput();

    static const double PID_KP;
    static const double PID_KI;
    static const double PID_KD;
    static const double GYRO_MULT_PITCH;
    static const double GYRO_MULT_ROLL;
    static const double GYRO_MULT_YAW;
};

    const double StabilityController::PID_KP =            4.00; // proportional
    const double StabilityController::PID_KI =            1.00; // integral
    const double StabilityController::PID_KD =            0.00; // derivative
    const double StabilityController::GYRO_MULT_PITCH =   0.50; // influence of gyro on output
    const double StabilityController::GYRO_MULT_ROLL =    0.50;
    const double StabilityController::GYRO_MULT_YAW =     0.50;

StabilityController::StabilityController(RemoteControl* rc, ROSController* ros, IMUController *imu) {
  this->_rc = rc;
  this->_ros = ros;
  this->_imu = imu;
  
  this->_PIDPitchPos = new PID(&this->_pid_pitchp_input, 
                            &this->_pid_pitchp_output, 
                            &this->_pid_pitchp_setpoint, 
                            StabilityController::PID_KP, 
                            StabilityController::PID_KI, 
                            StabilityController::PID_KD, 
                            DIRECT
                          );
  this->_PIDRollPos = new PID(&this->_pid_rollp_input, 
                             &this->_pid_rollp_output, 
                             &this->_pid_rollp_setpoint, 
                             StabilityController::PID_KP, 
                             StabilityController::PID_KI, 
                             StabilityController::PID_KD, 
                             DIRECT
                           );
  this->_PIDPitchNeg = new PID(&this->_pid_pitchn_input, 
                            &this->_pid_pitchn_output, 
                            &this->_pid_pitchn_setpoint, 
                            StabilityController::PID_KP, 
                            StabilityController::PID_KI, 
                            StabilityController::PID_KD, 
                            DIRECT
                          );
  this->_PIDRollNeg = new PID(&this->_pid_rolln_input, 
                             &this->_pid_rolln_output, 
                             &this->_pid_rolln_setpoint, 
                             StabilityController::PID_KP, 
                             StabilityController::PID_KI, 
                             StabilityController::PID_KD, 
                             DIRECT
                           );

  this->_PIDPitchPos->SetMode(AUTOMATIC);
  this->_PIDPitchNeg->SetMode(AUTOMATIC);
  this->_PIDRollPos->SetMode(AUTOMATIC);
  this->_PIDRollNeg->SetMode(AUTOMATIC);
}

/**
 * output range -1 to 1 for each
 */
double* StabilityController::getPRYOutput() {
   /**
   * get gyro output
   */

  double gyro_x, gyro_y, gyro_z;
  double* o = this->_imu->getGroundedGyroOutput();
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
   * get PRY output
   */
   
  double ppos = this->_pid_pitchp_output / PID_OUTPUT_MAX;
  double rpos = this->_pid_rollp_output / PID_OUTPUT_MAX;
  double pneg = this->_pid_pitchn_output / PID_OUTPUT_MAX;
  double rneg = this->_pid_rolln_output / PID_OUTPUT_MAX;
  double yaw = this->_r_yaw / YAW_MAX;

  double gyro_x_comp = gyro_x * StabilityController::GYRO_MULT_PITCH;
  double gyro_y_comp = gyro_y * StabilityController::GYRO_MULT_ROLL;
  double gyro_z_comp = gyro_z * StabilityController::GYRO_MULT_YAW;

  // add in the gyro values -jkr

  double p = ppos - pneg - gyro_x_comp;
  double r = rpos - rneg - gyro_y_comp;
  double y = yaw - gyro_z_comp;

//  override output to only gyro
//  p = 0 - gyro_x_comp;
//  r = 0 - gyro_y_comp;
//  y = 0 - gyro_z_comp;

  if(false) {
    Serial.print(gyro_x_comp);
    Serial.print(F("\t"));
    Serial.print(gyro_y_comp);
    Serial.print(F("\t"));
    Serial.println(gyro_z_comp);
  }

  // make sure values don't go outside of what is expected -jkr
  // commenting this out to pave way for a max buffering system in EC -jkr
  
//  if(p > 1)   p = 1;
//  if(p < -1)  p = -1;
//
//  if(r > 1)   r = 1;
//  if(r < -1)  r = -1;
//
//  if(y > 1)   y = 1;
//  if(y < -1)  y = -1;

  if(false) {
    Serial.print(p);
    Serial.print(F("\t"));
    Serial.print(r);
    Serial.print(F("\t"));
    Serial.println(y);
  }
  
  return new double[3] {p, r, y};
}

void StabilityController::loop() {

  /**
   * get desired drone position
   */
  
  double* o;
  double r_pitch = 0, r_roll = 0, r_yaw = 0;

  if(this->_ros->isOn()) {
    o = this->_ros->getAttitude();
    r_pitch = o[0];
    r_roll =  o[1];
    r_yaw =   o[2];
    delete [] o;
  }
  
  // rc takes precedence
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
  
  o = this->_imu->getFilteredAttitude();
  double imu_pitch, imu_roll, imu_yaw;
  imu_pitch = o[0];
  imu_roll  = o[1];
  imu_yaw   = o[2];
  delete [] o;

  if(true) {
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

  this->_pid_pitchp_input = imu_pitch;
  this->_pid_pitchn_input = r_pitch;
  this->_pid_rollp_input = imu_roll;
  this->_pid_rolln_input = r_roll;

  this->_pid_pitchp_setpoint = r_pitch;
  this->_pid_pitchn_setpoint = imu_pitch;
  this->_pid_rollp_setpoint = r_roll;
  this->_pid_rolln_setpoint = imu_roll;

  this->_r_yaw = r_yaw;

  // might need to change pid settings when we are close to the goal -jkr
  this->_PIDPitchPos->Compute();
  this->_PIDRollPos->Compute();
  this->_PIDPitchNeg->Compute();
  this->_PIDRollNeg->Compute();

  if(false) {// && this->_rc->isOn()) {
    Serial.print(F("sc pid roll: "));
    Serial.print(this->_pid_rollp_input);
    Serial.print(F(" "));
    Serial.print(this->_pid_rollp_setpoint);
    Serial.print(F(" "));
    Serial.print(this->_pid_rollp_output);
    Serial.print(F("\t\t"));
    Serial.print(this->_pid_rolln_input);
    Serial.print(F(" "));
    Serial.print(this->_pid_rolln_setpoint);
    Serial.print(F(" "));
    Serial.println(this->_pid_rolln_output);
  }

  // check the outputs
  if(false) {
    Serial.print(this->_pid_pitchp_output);
    Serial.print(F(" "));
    Serial.print(this->_pid_rollp_output);
    Serial.print(F("\t"));
    Serial.print(this->_pid_pitchn_output);
    Serial.print(F(" "));
    Serial.println(this->_pid_rolln_output);
  }
}

