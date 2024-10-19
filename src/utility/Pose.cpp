#include "utility/Pose.h"


Pose::Pose(float x, float y, float theta) {
  //constructor for pose object

  this->x = x;
  this->y = y;
  this->theta = theta;
}


Pose Pose::operator+(const Pose& other) {
  //returns the sum of two poses

  return Pose(this->x + other.x, this->y + other.y, this->theta);
}


Pose Pose::operator-(const Pose& other) {
  //returns the difference of two poses

  return Pose(this->x - other.x, this->y - other.y, this->theta);
}


Pose Pose::operator*(const Pose& other) {
  //returns the product of two poses

  return Pose(this->x * other.x, this->y * other.y, this->theta);
}


Pose Pose::operator/(const Pose& other) {
  //returns the dividend of two poses

  return Pose(this->x / other.x, this->y / other.y, this->theta);
}


Pose Pose::operator=(const Pose& other) {
  //sets a pose equal to the other pose

  return Pose(other.x, other.y, other.theta);
}


Pose Pose::operator+(const float& other) {
  //returns the sum of a pose and a constant

  return Pose(this->x + other, this->y + other, this->theta);
}


Pose Pose::operator-(const float& other) {
  //returns the difference of a pose and a constant

  return Pose(this->x - other, this->y - other, this->theta);
}


Pose Pose::operator*(const float& other) {
  //returns the product of a pose and a constant

  return Pose(this->x * other, this->y * other, this->theta);
}


Pose Pose::operator/(const float& other) {
  //returns the dividend of a pose and a constant

  return Pose(this->x / other, this->y / other, this->theta);
}


float Pose::dotProduct(Pose other) {
  //returns the dot product of two poses

  return this->x * other.x + this->y * other.y;
}


float Pose::distance(Pose other) {
  //returns the distance between two poses

  return Misc::getDistance(this->x, this->y, other.x, other.y);
}


float Pose::face(Pose other, bool rad) {
  //returns the angle one pose must turn to face the other pose

  float theta = Misc::reduceAngle(Units::radToDeg(atan2(other.x - this->x, other.y - this->y)) - this->theta);
  return (rad) ? Units::degToRad(theta) : theta;
}


float Pose::angle(Pose other, bool rad) {
  //returns the angle one pose must turn to face the other pose relative to the y-axis

  float theta = Misc::reduceAngle(Units::radToDeg(atan2(other.x - this->x, other.y - this->y)));
  return (rad) ? Units::degToRad(theta) : theta;
}


float Pose::parallel(Pose other) {
  //returns the angle one pose must turn to be parallel with another pose

  return Misc::reduceAngle(other.theta - this->theta);
}


float Pose::parallel(float target) {
  //returns the angle one pose must turn to face an angle

  return Misc::reduceAngle(target - this->theta);
}


void Pose::set(float x, float y, float theta) {
  //sets the pose data

  this->x = x;
  this->y = y;
  this->theta = theta;
}