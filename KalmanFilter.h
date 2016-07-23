/**
 * Jackson Rollins Jun 30 2016 12:02pm Provo, UT
 * 
 * I've rifled through a number os kalman filtersin JS, C, C++, and even an arduino cpp version.
 * I decided to convert one from C I found on the following site:
 * 
 * http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/
 * 
 */

class KalmanFilter {
  public:
    KalmanFilter(double q, double r, double p, double x);
    void loop(double m);
    double getX() {
      return this->_x;
    }
  private:
    double _q; //process noise covariance
    double _r; //measurement noise covariance
    double _p; //estimation error covariance
    double _x; //value
    double _k; //kalman gain
};

KalmanFilter::KalmanFilter(double q, double r, double p, double x) {
  this->_q = q;
  this->_r = r;
  this->_p = p;
  this->_x = x; // initial value
}

/**
 * @value m Measurement
 */
void KalmanFilter::loop(double m) {
  //prediction update
  //omit x = x
  this->_p += this->_q;

  //measurement update
  this->_k = this->_p / (_p + this->_r);
  this->_x = this->_x + _k * (m - this->_x);
  this->_p = (1 - this->_k) * this->_p;
}

