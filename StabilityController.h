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
    double* getPIDOutput();

    static const int PID_KP; 
    static const int PID_KI; 
    static const int PID_KD;
};

    const int StabilityController::PID_KP = 10; 
    const int StabilityController::PID_KI = 3; 
    const int StabilityController::PID_KD = 1;

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

double* StabilityController::getPIDOutput() {
  double ppos = this->_pid_pitchp_output / PID_OUTPUT_MAX;
  double rpos = this->_pid_rollp_output / PID_OUTPUT_MAX;
  double pneg = this->_pid_pitchn_output / PID_OUTPUT_MAX;
  double rneg = this->_pid_rolln_output / PID_OUTPUT_MAX;

  double p = ppos - pneg;
  double r = rpos - rneg;

  if(false) {
    Serial.print(ppos);
    Serial.print(F(" "));
    Serial.println(pneg);
  }
  
  return new double[2] {p, r};
}

void StabilityController::loop() {

  /**
   * get desired drone position
   */
  
  double* rAttitude;
  double r_pitch = 0, r_roll = 0, r_yaw = 0;

  if(this->_ros->isOn()) {
    rAttitude = this->_ros->getAttitude();
    r_pitch = rAttitude[0];
    r_roll =  rAttitude[1];
    r_yaw =   rAttitude[2];
    delete [] rAttitude;
  }
  
  // rc takes precedence
  if(this->_rc->isOn()) {
    rAttitude = this->_rc->getAttitude();
    r_pitch = rAttitude[0];
    r_roll =  rAttitude[1];
    r_yaw =   rAttitude[2];
    delete [] rAttitude;
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
  
  double* imuAttitude = this->_imu->getAttitude();
  double imu_pitch, imu_roll, imu_yaw;
  imu_pitch = imuAttitude[0];
  imu_roll  = imuAttitude[1];
  imu_yaw   = imuAttitude[2];
//  delete [] imuAttitude;

  if(false) {
    Serial.print("imu: ");
    Serial.print(imu_pitch);
    Serial.print(" ");
    Serial.print(imu_roll);
    Serial.print(" ");
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

