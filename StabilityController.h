#include <PID_v1.h>

#define PID_OUTPUT_MAX 255;

class StabilityController {
  protected:
    RemoteControl* _rc;
    ROSController* _ros;
    IMUController* _imu;
    PID* _PIDPitch;
    PID* _PIDRoll;
    double _pid_pitch_input, _pid_pitch_output, _pid_pitch_setpoint;
    double _pid_roll_input, _pid_roll_output, _pid_roll_setpoint;
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

    const int StabilityController::PID_KP = 4; 
    const int StabilityController::PID_KI = 10; 
    const int StabilityController::PID_KD = 2;

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

  this->_PIDPitch->SetMode(AUTOMATIC);
  this->_PIDRoll->SetMode(AUTOMATIC);
}

double* StabilityController::getPIDOutput() {
  double p = this->_pid_pitch_output / PID_OUTPUT_MAX;
  double r = this->_pid_roll_output / PID_OUTPUT_MAX;
  
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
   * get current drone position
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
   * compare desired vs current to get error
   */

   // !! I think PID finds the error already? -jkr

//   // might to use math.abs
//   double p_error, r_error;
//   p_error = rc_pitch - imu_pitch;
//   r_error = rc_roll - imu_roll;
//
//   if(false && this->_rc->isOn()) {
//     Serial.print(p_error);
//     Serial.print(" ");
//     Serial.println(r_error);
//   }

  /** 
   * send error to PID 
   */

  this->_pid_pitch_input = imu_pitch;
  this->_pid_roll_input = imu_roll;

  this->_pid_pitch_setpoint = r_pitch;
  this->_pid_roll_setpoint = r_roll;

  // might need to change pid settings when we are close to the goal -jkr
  this->_PIDPitch->Compute();
  this->_PIDRoll->Compute();

  if(false) {// && this->_rc->isOn()) {
    Serial.print("sc pid roll: ");
    Serial.print(this->_pid_roll_input);
    Serial.print(" ");
    Serial.print(this->_pid_roll_setpoint);
    Serial.print(" ");
    Serial.println(this->_pid_roll_output);
  }

  // check the outputs
  if(false) {
    Serial.print(this->_pid_pitch_output);
    Serial.print(" ");
    Serial.println(this->_pid_roll_output);
  }
}

