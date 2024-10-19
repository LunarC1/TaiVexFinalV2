#include "utility/AsymptoticGains.h"


AsymptoticGains::AsymptoticGains(float i, float f, float k, float p) {
  //constructor for asymptotic gain object

  this->i = i;
  this->f = f;
  this->k = k;
  this->p = p;
}


void AsymptoticGains::setGain(float setpoint) {
  //sets the gain value based on the setpoint

  this->setpoint = setpoint;
}


float AsymptoticGains::getGain() {
  //returns the gain value

  return (f - i) * pow(fabs(setpoint), p) / ((pow(fabs(setpoint), p)) + pow(k, p)) + i;
}