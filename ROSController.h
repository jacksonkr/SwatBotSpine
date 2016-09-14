//#include <ros.h> // https://github.com/ros-drivers/rosserial/blob/jade-devel/rosserial_arduino/src/ros_lib/ros.h
//#include <std_msgs/String.h>

class ROSController {
  protected:
//    std_msgs::String str_msg;
//    ros::NodeHandle _nh;
    int _throttle;
    bool _is_on = false;
  public:
    void loop();
    ROSController();
    double* getAttitude();
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

//  ros::Publisher chatter("chatter", &str_msg);
  
//  this->_nh.initNode();
//  this->_nh.advertise(chatter);
}

double* ROSController::getAttitude() {
  return new double[3]{0, 0, 0};
}

void ROSController::loop() {
//  Serial.print(F("ROS loop "));
//  Serial.println(millis());
  
//  str_msg.data = new char[] {"hello"};
//  chatter.publish( str_msg );
//  this->_nh.spinOnce();
}

