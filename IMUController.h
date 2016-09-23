/**
 * NOTES
 * Orientation of Adafruit LSM303 matchs by the orientation of the font on the board - nose up is positive pitch, left wing up is positive roll
 * 
 */

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
//#include <KalmanFilter.h>
#include <Kalman.h>

class IMUController {
  protected:
    bool _initialized = false;
    JacksonSimpleFilter _jsfPitch;
    JacksonSimpleFilter _jsfRoll;
    JacksonSimpleFilter _jsfHeading;
    void adustForFront(float*); //todo: pass arr[3] of xyz or pry and adust the xy/pr for where the front_is_facing -jkr
    int _set_ground_loop_count = 0;
    /**
     * handle set_ground using the constructor, code is already there.
     */
    int _set_ground = 0;
    double _groundPR[2] = {0, 0};
    /**
     * substracted from gyro outputs to zero them out
     * todo: make this dynamic and potentially apply kalman -jkr
     */
    float _groundGyro[3] = {-0.057, 0.0315, 0.075}; // relative to the chip
    sensor_t sensor;
    Adafruit_10DOF                dof   = Adafruit_10DOF();
    Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
    Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
    Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
    Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    double _attitude[3];
    double _gyro[3];
    void getGroundedPR(float*);
    void setGroundedPR(float, float);
    void setFront(int);
  public:
    static int front_is_facing;
    
    IMUController();
    float* getAttitude();
    void setAttitude(float, float, float);
    void setGyro(double, double, double);
    bool getInitialized();
    float* getGyro();
    void loop();
};

// 1, 1 is nose up and right wing down // https://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft) -jkr
int IMUController::front_is_facing = -180; // changeable by UI (when there is one) -jkr

IMUController::IMUController() {

  gyro.enableAutoRange(false);

  if (false) { // debug set grounded !! WRITES TO EEPROM -jkr
    this->_set_ground = 1;
    Serial.println("!! SET GROUNDED IS ON !!  IMUController::set_ground = true");\
  }

  if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }

  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  
  if (!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while (1);
  }
  
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }

  accel.getSensor(&sensor);
  gyro.getSensor(&sensor);
  mag.getSensor(&sensor);
  bmp.getSensor(&sensor);
}

/**
 * returns attitude both filtered and normalized (grounded)
 */
float* IMUController::getAttitude() {
  float p, r, y;

  // 90 same
  // -180 switch both, reverse roll
  // 270 switch both, reverse both
  // 0 / 360 switch both, reverse pitch
  if(front_is_facing == -180) {
    p = (this->_attitude[1] - this->_groundPR[1]) * -1; // convert pitch to roll
    r = this->_attitude[0] - this->_groundPR[0]; // convert roll to pitch and reverse it
  } else if(front_is_facing == 90) { // normal layout
    p = this->_attitude[1] - this->_groundPR[1];
    r = this->_attitude[0] - this->_groundPR[0];
  }
  y = this->_attitude[2];

  if(false) {
    Serial.print(F("IMUController->getAttitude: "));
    Serial.print(p);
    Serial.print(F(", "));
    Serial.print(r);
    Serial.print(F(", "));
    Serial.println(y);
  }
  
  return new float[3]{p, r, y};
}

/**
 * attitudes are set based on objective chip orientation. "front_is_facing" is only applied to output (getAttitdue() output) -jkr
 */
void IMUController::setAttitude(float p, float r, float y) {
  if(true) {
    Serial.print(F("IMUController->setAttitude: ")); // accounts for hardware, not for "front_is_facing" yet. -jkr
    Serial.print(p);
    Serial.print(", ");
    Serial.print(r);
    Serial.print(", ");
    Serial.println(y);
  }
  
  this->_attitude[0] = p;
  this->_attitude[1] = r;
  this->_attitude[2] = y;
}

void IMUController::setGyro(double x, double y, double z) {
  if(false) {
    Serial.print(F("IMU->setGyro "));
    Serial.print(x);
    Serial.print(F(" "));
    Serial.print(y);
    Serial.print(F(" "));
    Serial.println(z);
  }
  
  this->_gyro[0] = x;
  this->_gyro[1] = y;
  this->_gyro[2] = z;
}

/**
 * returns gyro normalized (grounded)
 * unbiased raw values: pitch(x) nose thrust up is + and roll(y) left wing thrust up is +
 */
float* IMUController::getGyro() {
  double x, y, z;

  if(false) { // not-grounded not-normalized
    Serial.print(this->_gyro[0]);
    Serial.print(F(" "));
    Serial.print(this->_gyro[1]);
    Serial.print(F(" "));
    Serial.println(this->_gyro[2]);
  }

  if(IMUController::front_is_facing == -180) { // left a quarter turn
    x = (this->_gyro[1] - this->_groundGyro[1]) * -1;
    y = this->_gyro[0] - this->_groundGyro[0];
  } else if(IMUController::front_is_facing == 90) { // normal
    x = this->_gyro[0] - this->_groundGyro[0];
    y = this->_gyro[1] - this->_groundGyro[1];
  }
  z = this->_gyro[2] - this->_groundGyro[2];

  if(false) { // normalized
    Serial.print(x);
    Serial.print(F(" "));
    Serial.print(y);
    Serial.print(F(" "));
    Serial.println(z);
  }

  return new float[3]{x, y, z};
}

bool IMUController::getInitialized() {
  float g[2] = {0, 0};
  this->getGroundedPR(g);

  if (false) {
    Serial.print(F("IMU ground values: "));
    Serial.print(g[0]);
    Serial.print(F(" "));
    Serial.println(g[1]);
  }

  if (g[0] == 0 || g[1] == 0) return false;
  this->_groundPR[0] = g[0];
  this->_groundPR[1] = g[1];

  if (false) {
    Serial.print(F("grounded IMU values: "));
    Serial.print(this->_groundPR[0]);
    Serial.print(F(" "));
    Serial.println(this->_groundPR[1]);
  }
  
  return true;
}

void IMUController::getGroundedPR(float* g) {
  float ret[2] = {0, 0};

  EEPROM.get(EEPROM_ADDRESS_IMU_GROUNDED, ret);

  g[0] = ret[0];
  g[1] = ret[1];
  // memcpy(g, ret, 2);
}
void IMUController::setGroundedPR(float p, float r) {
  float pr[2] = {p, r};
  
  Serial.print(F("Setting IMU ground: "));
  Serial.print(p);
  Serial.print(F(" "));
  Serial.println(r);
  EEPROM.put(EEPROM_ADDRESS_IMU_GROUNDED, pr);
}
void IMUController::setFront(int angle) {
  Serial.print("Setting front: ");

  // right now this is set to a const on the .ino

  Serial.println();
//    EEPROM.write(EEPROM_ADDRESS_IMU_FRONT, arr);
}

void IMUController::loop() {
  /* Get a new sensor event */
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_event_t gyro_event; 
  sensors_vec_t   orientation;

  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  gyro.getEvent(&gyro_event);

  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {

    // values of attitude before transforming for position of "front_is_facing" (unbiased attitude) -jkr
    if(false) { // unbiased attitude debug
      Serial.print(orientation.pitch);
      Serial.print(F(" "));
      Serial.print(orientation.roll);
      Serial.print(F(" "));
      Serial.println(orientation.heading);
    }
    
    /**
       set what the IMU is reporting when it's grounded & motors off
    */
    
    // !! pitch/roll are switched for LSM303 -jkr
    float pv = orientation.pitch;
    orientation.pitch = orientation.roll;
    orientation.roll = pv;

    if(this->_initialized == false) {
      this->_initialized = true;
      this->_jsfPitch.setInitialValue(orientation.pitch);
      this->_jsfRoll.setInitialValue(orientation.roll);
      this->_jsfHeading.setInitialValue(orientation.heading);
    } else {
      this->_jsfPitch.shiftValue(orientation.pitch);
      this->_jsfRoll.shiftValue(orientation.roll);
      this->_jsfHeading.shiftValue(orientation.heading);
        
      if (this->_set_ground == 1) {
        if (orientation.pitch != 0.00 && orientation.roll != 0.00) {
  //        Serial.println("set ground kalman init");
          this->_set_ground = 2;
        }
      }
      //Serial.println(this->_kalmanGroundPitch->getX());
      if (this->_set_ground == 2) {
        if (++this->_set_ground_loop_count > 100) 
          this->_set_ground = 3;
      }
  
      if (this->_set_ground == 3) {
        this->_set_ground = -1;
          
        this->setGroundedPR(this->_jsfPitch.getFilterValue(), this->_jsfRoll.getFilterValue());
        
        Serial.print(F("set ground kalman finish: "));
        Serial.print(orientation.pitch);
        Serial.print(F(" "));
        Serial.println(this->_jsfPitch.getFilterValue());
        Serial.println(F("!!"));
        Serial.println(F("!! NOW STOP WRITING TO THE EEPROM BEFORE YOU RUIN IT, QUITTING PROGRAM"));
        Serial.println(F("!!"));
  
        while(1);
      }

      if(false) { // debug
        Serial.print(orientation.pitch);
        Serial.print(F(" "));
//        Serial.println(kp);
        Serial.println(this->_jsfPitch.getFilterValue());
      }
      
      this->setAttitude(this->_jsfPitch.getFilterValue(), this->_jsfRoll.getFilterValue(), this->_jsfHeading.getFilterValue());
    }

    // get gyro info -jkr

    if(false) {
      Serial.print(F("gyro: "));
      Serial.print(gyro_event.gyro.x);
      Serial.print(F("\t"));
      Serial.print(gyro_event.gyro.y);
      Serial.print(F("\t"));
      Serial.println(gyro_event.gyro.z);
    }

    this->setGyro(gyro_event.gyro.x, gyro_event.gyro.y, gyro_event.gyro.z);
  }

  if (false) { // barometric pressure debug

    bmp.getEvent(&bmp_event);
    if (bmp_event.pressure)
    {
      /* Get ambient temperature in C */
      float temperature;
      bmp.getTemperature(&temperature);
      /* Convert atmospheric pressure, SLP and temp to altitude */
      Serial.print(F("Alt: "));
      Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                          bmp_event.pressure,
                                          temperature));
      Serial.println(F(""));
      /* Display the temperature */
      Serial.print(F("Temp: "));
      Serial.print(temperature);
      Serial.println(F(""));
    }

  }
}

