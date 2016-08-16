#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

class IMUController {
  protected:
    sensor_t sensor;
    Adafruit_10DOF                dof   = Adafruit_10DOF();
    Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
    Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
    Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
    Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    double _attitude[3];
  public:
    static IMUController* instance;
    IMUController();
    void setAttitude(double p, double r, double y) {
      this->_attitude[0] = p;
      this->_attitude[1] = r;
      this->_attitude[2] = y;
    }
    double* getAttitude() {
      return this->_attitude;
    }
    void loop();
};

IMUController* IMUController::instance = NULL;

IMUController::IMUController() {
  if (IMUController::instance != NULL) {
    Serial.print(ERROR_STR);
    Serial.println("IMU is a singleton");
    while (1);
  }
  IMUController::instance = this;

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
    this->setAttitude(orientation.pitch, orientation.roll, orientation.heading);

    if (false) { // attitude debug
      /* 'orientation' should have valid .roll and .pitch fields */
      Serial.print(F("Orientation: "));
      Serial.print(orientation.roll);
      Serial.print(F(" "));
      Serial.print(orientation.pitch);
      Serial.print(F(" "));
      Serial.print(orientation.heading);
      Serial.println(F(""));
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

