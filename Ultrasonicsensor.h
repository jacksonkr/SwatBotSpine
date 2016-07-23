class UltrasonicSensor {
  public:
    
    UltrasonicSensor(int trigPin, int echoPin);
    void loop();
    double read() {
      return _distance;
    }
  private:
    const int _TIMEOUT = 30; // milliseconds
    const float _SPEED_OF_SOUND = 1249.7;// kmh 776.5; // mph

    void error();
    int _trigPin;
    int _echoPin;
    float _pingTime;
    double _distance;
    KalmanFilter* _filter;
};

UltrasonicSensor::UltrasonicSensor(int trigPin, int echoPin) {
  this->_trigPin = trigPin;
  this->_echoPin = echoPin;
  this->_filter = new KalmanFilter(40000, 1, 1, 0);

  pinMode(this->_trigPin, OUTPUT);
  pinMode(this->_echoPin, INPUT);
}

void UltrasonicSensor::loop() {
  int latency = millis();
  
  digitalWrite(this->_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(this->_trigPin, HIGH);
  delayMicroseconds(5);

  this->_pingTime = pulseIn(this->_echoPin, HIGH, UltrasonicSensor::_TIMEOUT * 1000);
  this->_pingTime /= 1000000; // microseconds to seconds
  this->_pingTime /= 3600; // hours

  latency = millis() - latency;
//  Serial.println("latency: " + (String)latency);

  float d = UltrasonicSensor::_SPEED_OF_SOUND * this->_pingTime; // km OR miles depending on settings
  d /= 2; // to <> from target (averaging distance)
  //d *= 63360; // miles to inches
  d *= 1000000;

  if(this->_pingTime <= 0 || latency >= UltrasonicSensor::_TIMEOUT) { // 5 meters
    Serial.print(ERROR_STR);
    Serial.print(" - ");
    Serial.print(d);
    Serial.print(", latency: ");
    Serial.println((String)latency);

    this->_distance = -1;
    return;
  }
  
  this->_filter->loop(d);
  this->_distance = this->_filter->getX(); // return filtered distance
}

