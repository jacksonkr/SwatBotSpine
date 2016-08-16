class ROSController {
  protected:
    void loop();
    int _throttle;
    bool _is_on = false;
  public:
    ROSController();
    bool isOn() {
      return this->_is_on;
    }
    int getThrottlePerc() {
      return this->_throttle / ROSController::THR_MAX;
    }
    
    static const int THR_MAX;
};

const int ROSController::THR_MAX = 100;

ROSController::ROSController() {
  
}

void ROSController::loop() {
  
}

