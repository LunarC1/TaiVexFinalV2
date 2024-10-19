#include "utility/Misc.h"
#include "utility/Pose.h"


float Misc::getHeading(bool reverse) {
  //returns average absolute angle of two imus
  
  //return (reverse) ? 0.5 * (imu.rotation(vex::deg) + imu2.rotation(vex::deg)) + 180.0 : 0.5 * (imu.rotation(vex::deg) + imu2.rotation(vex::deg));
  return (reverse) ? reduceAngle(imu.rotation(vex::deg) + 180.0) : reduceAngle(imu.rotation(vex::deg));
}


void Misc::runDrivetrain(float left, float right, bool reverse) {
  
  if (reverse) {
    leftDrivetrain.spin(vex::fwd, -right, vex::voltageUnits::mV);
    rightDrivetrain.spin(vex::fwd, -left, vex::voltageUnits::mV);
  }
  
  else {
    leftDrivetrain.spin(vex::fwd, left, vex::voltageUnits::mV); 
    rightDrivetrain.spin(vex::fwd, right, vex::voltageUnits::mV);
  }
}


float Misc::clamp(float input, float min, float max) {
  //caps output at a given absolute value maximum, retains sign

  if (input < min) return min;
  else if (input > max) return max;
  return input;
}


float Misc::getDistance(float x1, float y1, float x2, float y2) {
  //finds the distance between two points

  return sqrt(pow(x1 - x2, 2.0) + pow(y1 - y2, 2.0));
}


float Misc::reduceAngle(float theta) {
  //reduces an angular difference to its smallest absolute value

  return theta - 360.0 * std::floor((theta + 180.0) / 360.0);
}


float Misc::angleError(float target, float theta) {

  return reduceAngle(target - theta);
}


std::pair<float, float> Misc::scaleToRatio(float factor, float num1, float num2) {

  float ratio = factor / std::fmax(fabs(num1), fabs(num2));
  return std::make_pair(num1 * ratio, num2 * ratio);
}


std::pair<float, float> Misc::reduceRatio(float max, float num1, float num2) {
  //reduces a ratio between two numbers so that the absolute value of the larger number is equal to max

  if (fabs(num1) > fabs(max) && fabs(num1) >= fabs(num2)) {
    num2 *= max / fabs(num1);
    num1 = copysign(1.0, num1) * max;
  }
  else if (fabs(num2) > fabs(max) && fabs(num2) >= fabs(num1)) {
    num1 *= max / fabs(num2);
    num2 = copysign(1.0, num2) * max;
  }

  std::pair<float, float> nums = std::make_pair(num1, num2);
  return nums;
}





float Misc::getCurvature(Pose p1, Pose p2) {
  //finds the curvature between two points given a specified theta at p1
  
  float side = copysign(1.0, cos(Units::degToRad(p1.theta)) * (p2.x - p1.x) - sin(Units::degToRad(p1.theta)) * (p2.y - p1.y));
  float radius = getRadius(p1, p2);
  return (radius != 0) ? side / radius : std::numeric_limits<float>::max();
}


float Misc::getCurvature(Pose p1, Pose p2, float heading) {
  //finds the curvature between two points given a specified theta

  float side = copysign(1.0, cos(Units::degToRad(heading)) * (p2.x - p1.x) - sin(Units::degToRad(heading)) * (p2.y - p1.y));
  float radius = getRadius(p1, p2);
  return (radius != 0) ? side / radius : std::numeric_limits<float>::max();
}


float Misc::getCurvature(Pose p1, Pose p2, Pose p3) {
  //finds the curvature of the circle that intersects three points

  float side = copysign(1.0, (p2.y - p1.y)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.y - p1.y));
  float radius = getRadius(p1, p2, p3);
  printf("%f | %f\n", radius, 1/ radius);
  return (radius != 0) ? side / radius : std::numeric_limits<float>::max();
}


float Misc::getRadius(Pose p1, Pose p2) {
  //finds the radius of the circle that intersects two points given a specified theta at p1

  return p1.distance(p2) / (sqrt(2.0 - 2.0 * cos(2.0 * p1.face(p2, true))));
}


float Misc::getRadius(Pose p1, Pose p2, Pose p3) {
  //finds the radius of the circle that intersects three points

  float A = p1.x * (p2.y - p3.y) - p1.y * (p2.x - p3.x) + p2.x * p3.y - p3.x * p2.y;
  float B = (p1.x * p1.x + p1.y * p1.y) * (p3.y - p2.y) + (p2.x * p2.x + p2.y * p2.y) * (p1.y - p3.y) + (p3.x * p3.x + p3.y * p3.y) * (p2.y - p1.y);
  float C = (p1.x * p1.x + p1.y * p1.y) * (p2.x - p3.x) + (p2.x * p2.x + p2.y * p2.y) * (p3.x - p1.x) + (p3.x * p3.x + p3.y * p3.y) * (p1.x - p2.x);
  float D = (p1.x * p1.x + p1.y * p1.y) * (p3.x * p2.y - p2.x * p3.y) + (p2.x * p2.x + p2.y * p2.y) * (p1.x * p3.y - p3.x * p1.y) + (p3.x * p3.x + p3.y * p3.y) * (p2.x * p1.y - p1.x * p2.y);

  return sqrt((B * B + C * C - 4 * A * D) / (4 * A * A));
}


float Misc::circleIntersect(Pose linep1, Pose linep2, Pose center, float radius) {
  //finds the fractional index of the point of intersection between a circle and a line segment

  Pose d = linep2 - linep1;
  Pose f = linep1 - center;

  float a = d.dotProduct(d);
  float b = 2 * f.dotProduct(d);
  float c = f.dotProduct(f) - radius * radius;

  float discriminant = b * b - 4 * a * c;

  if (discriminant >= 0) {
    
    discriminant = sqrt(discriminant);
    float t1 = (-b - discriminant) / (2 * a);
    float t2 = (-b + discriminant) / (2 * a);
    
    if (t1 >= 0 && t1 <= 1) return t1;
    else if (t2 >= 0 && t2 <= 1) return t2;
  }

  return -1;
}


Pose Misc::lerp(Pose p1, Pose p2, float t) {
  //returns a point given two ends of a line segment and a fractional index

  // return Pose(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y));
  return p1 + (Pose(p2 - p1) * t);
}