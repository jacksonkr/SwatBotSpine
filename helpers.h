

// this isn't working when it's in helpers.cpp ?? It may have originally been some sort of memory issue.. -jkr
void arrayShift(double* arr, double val, int len) {
  memmove(&arr[1], &arr[0], (len-1)*sizeof(double));
  arr[0] = val;
}

/**
 * finds the array min, array max, and then calculates spread to make the closest gap to reach 100%
 * eg. arrayStretchTo(someArr, -100, 100); 
 * eg. arrayStretchTo({0,1,2,3}, 0, 6); // [0, 3, 4, 6]
 * todo: unfinished (out of time) -jkr
 */
void arrayStretchTo(double* arr, double min_cap, double max_cap) {
  int len = sizeof(arr) / sizeof(double);
  double min = 0;
  double max = 0;

  for(int i = 0; i < len; ++i) {
    if(arr[i] < min) min = arr[i];
    if(arr[i] > max) max = arr[i];
  }

  double distance = 0;
  if(abs(min_cap - min) < abs(max_cap - max)) {
    // max is closer
    distance = max_cap - max;
  } else {
    // min is closer
    distance = min_cap + min;
  }

  Serial.println(distance);

  // apply transformation
  for(int i = 0; i < len; ++i) {
    arr[i] *= distance;
  }
}

void arrayToSerial(double* arr) {
  Serial.print(F("{ "));
  
  for(int i=0; i < sizeof(arr) / sizeof(double); ++i) {
    Serial.print(i);
    Serial.print(F(", "));
    Serial.print(arr[i]);
  }
  
  Serial.println(F(" }"));
}

/**
 * Jackson Filter
 */

class JacksonSimpleFilter {
  private:
    float arr[20];
    int getArrLen();
  public:
    JacksonSimpleFilter();
    void setInitialValue(float);
    void shiftValue(float);
    float getFilterValue();
};

JacksonSimpleFilter::JacksonSimpleFilter() {
  //
}

void JacksonSimpleFilter::setInitialValue(float val) {
  this->arr[0] = val;
}

void JacksonSimpleFilter::shiftValue(float val) {
  memmove(&this->arr[1], &this->arr[0], (getArrLen()-1)*sizeof(float));
  this->arr[0] = val;
}

int JacksonSimpleFilter::getArrLen() {
  return sizeof(this->arr) / sizeof(float);
}

float JacksonSimpleFilter::getFilterValue() {
  float val = 0;
  for(int i = 0; i < getArrLen(); ++i) {
//    Serial.print(arr[i]);
//    Serial.print(F(" "));
    val += this->arr[i];
  }
//  Serial.println(F(" "));

  return val / 20;
}
