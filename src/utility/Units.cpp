#include "utility/Units.h"


float Units::radToDeg(float theta) {
  //converts a specified number of radians to degrees

  return theta * 180.0 / M_PI;
}


float Units::degToRad(float theta) {
  //converts a specified number of degrees to radians

  return theta * M_PI / 180.0;
}


float Units::tickToUnit(float num, float wheelDiameter, float unit) {
  //converts from ticks to a unit

  return num * wheelDiameter * M_PI / 360.0 * unit;
}


float Units::unitToTick(float num, float wheelDiameter, float unit) {
  //converts from a unit to ticks

  return num / wheelDiameter / M_PI * 360.0 / unit;
}


float Units::unitToUnit(float num, float unit1, float unit2) {
  //converts a unit to another unit

  return num / unit1 * unit2;
}