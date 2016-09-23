#ifdef ROS_C_ENABLED

#include <ros.h> // https://github.com/ros-drivers/rosserial/blob/jade-devel/rosserial_arduino/src/ros_lib/ros.h
#include <std_msgs/Empty.h>

class ROSController {
  protected:
    ros::Subscriber<std_msgs::Empty> sub;
    ros::NodeHandle _nh;
    static void setPRYCallback( const std_msgs::Empty& );
    int _throttle;
    bool _is_on = false;
  public:
    ROSController();
    void loop();
    float* getAttitude();
    bool isOn() {
      return this->_is_on;
    }
    int getThrottlePerc() {
      return this->_throttle / ROSController::THR_MAX;
    }
    
    static const int THR_MAX;
};

const int ROSController::THR_MAX = 100;

void ROSController::setPRYCallback( const std_msgs::Empty& toggle_msg) {}

ROSController::ROSController() : sub("setPRY", &setPRYCallback )  {
  this->_nh.initNode();
  this->_nh.subscribe(sub);
}

float* ROSController::getAttitude() {
  return new float[3]{0, 0, 0};
}

void ROSController::loop() {
//  Serial.print(F("ROS loop "));
//  Serial.println(millis());

  this->_nh.spinOnce();
  delay(1000);
}

#else
// dummy controller to appease the compiler
class ROSController {
  public:
    ROSController();
    bool isOn() { return false; };
    float* getAttitude() { return new float[3] {0, 0, 0}; };
    int getThrottlePerc() { return 0; };
};
ROSController::ROSController() {}
#endif
