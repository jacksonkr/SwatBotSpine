#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

class IMUController {
  protected:
    bool _initialized = false;
    KalmanFilter _kalmanPitch = KalmanFilter();
    KalmanFilter _kalmanRoll = KalmanFilter();
    KalmanFilter _kalmanHeading = KalmanFilter();
    int _set_ground_loop_count = 0;
    int _set_ground = 0;
    double _groundPR[2] = {0, 0};
    sensor_t sensor;
    Adafruit_10DOF                dof   = Adafruit_10DOF();
    Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
    Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
    Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
    Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    double _attitude[3];
    void getGroundedPR(double*);
    void setGroundedPR(double, double);
    void setFront(int);
  public:
    IMUController();
    void setAttitude(double p, double r, double y) {
      if(FRONT == 90) {
        if(this->_groundPR[0] != 0.00) r -= this->_groundPR[0];
        if(this->_groundPR[1] != 0.00) p -= this->_groundPR[1];
      }

      if(false) {
        Serial.print("IMU->setAttitude\t");
        Serial.print(p);
        Serial.print("\t");
        Serial.print(r);
        Serial.print("\t");
        Serial.println(y);
      }
      
      this->_attitude[0] = p;
      this->_attitude[1] = r;
      this->_attitude[2] = y;
    }
    bool getInitialized();
    double* getAttitude() {
      return this->_attitude;
    }
    void loop();
};

IMUController::IMUController() {

  if (false) { // debug set grounded
    this->_set_ground = 1;
    Serial.println("!! SET GROUNDED IS ON !!  IMUController::set_ground = true");
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

  accel.getSensor(&sensor);
  gyro.getSensor(&sensor);
  mag.getSensor(&sensor);
  bmp.getSensor(&sensor);
}

bool IMUController::getInitialized() {
  double g[2] = {0, 0};
  this->getGroundedPR(g);

  if (false) {
    Serial.print("Getting IMU ground: ");
    Serial.print(g[0]);
    Serial.print(" ");
    Serial.println(g[1]);
  }

  if (g[0] == 0 || g[1] == 0) return false;
  this->_groundPR[0] = g[0];
  this->_groundPR[1] = g[1];
  return true;
}

void IMUController::getGroundedPR(double* g) {
  double ret[2] = {0, 0};

  EEPROM.get(EEPROM_ADDRESS_IMU_GROUNDED, ret);

  g[0] = ret[0];
  g[1] = ret[1];
  // memcpy(g, ret, 2);
}
void IMUController::setGroundedPR(double p, double r) {
  Serial.print("Setting IMU ground: ");

  double pr[2] = {p, r};
  Serial.print(p);
  Serial.print(" ");
  Serial.println(r);
  EEPROM.put(EEPROM_ADDRESS_IMU_GROUNDED, pr);
}
void IMUController::setFront(int angle) {
  Serial.print("Setting front: ");

  // right now this is set to a const on the .ino

  Serial.println();
  //  EEPROM.write(EEPROM_ADDRESS_IMU_FRONT, arr);
}

void IMUController::loop() {
  /* Get a new sensor event */
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    /**
       set what the IMU is reporting when it's grounded & motors off
    */
    if(this->_initialized == false) {
      this->_initialized = true;
      this->_kalmanPitch.setValue(orientation.pitch);
      this->_kalmanRoll.setValue(orientation.roll);
      this->_kalmanHeading.setValue(orientation.heading);
    } else {
      this->_kalmanPitch.loop(orientation.pitch);
      this->_kalmanRoll.loop(orientation.roll);
      this->_kalmanHeading.loop(orientation.heading);
    }
    if (this->_set_ground == 1) {
      if (orientation.pitch != 0.00 && orientation.roll != 0.00) {
        Serial.println("!! set ground kalman init");
        this->_set_ground = 2;
      }
    }
    //Serial.println(this->_kalmanGroundPitch->getX());
    if (this->_set_ground == 2) {
      if (false) {
        Serial.print("!! set ground kalman update ");
        Serial.print(_set_ground_loop_count);
        Serial.print(" ");
        Serial.print(orientation.pitch);
        Serial.print(" ");
        Serial.print(this->_kalmanPitch.getX());
        Serial.println();
      }

      if (++this->_set_ground_loop_count > 100)
        this->_set_ground = 3;
    }

    if (this->_set_ground == 3) {
      this->_set_ground = -1;
      if (false) {
        Serial.print("!! set ground kalman finish ");
        Serial.print(orientation.pitch);
        Serial.print(" ");
        Serial.print(this->_kalmanPitch.getX());
        Serial.println();
      }
      this->setGroundedPR(this->_kalmanPitch.getX(), this->_kalmanRoll.getX());
    }

    if (false) { // raw attitude debug
      /* 'orientation' should have valid .roll and .pitch fields */
      Serial.print(F("Orientation: "));
      Serial.print(orientation.pitch);
      Serial.print(F(" "));
      Serial.print(orientation.roll);
      Serial.print(F(" "));
      Serial.print(orientation.heading);
      Serial.println(F(""));
    }

    // using raw attitudes
//    this->setAttitude(orientation.pitch, orientation.roll, orientation.heading);
    // using kalman filter
//    this->setAttitude(this->_kalmanPitch.getX(), this->_kalmanRoll.getX(), this->_kalmanHeading.getX());

    //todo: make this dynamic
    if(FRONT == 90) {
      this->setAttitude(this->_kalmanRoll.getX(), this->_kalmanPitch.getX(), this->_kalmanHeading.getX());
    }
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

