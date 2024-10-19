#include "components/MotorGroup.h"


MotorGroup::MotorGroup(std::vector<Motor> motors) {
  //constructor: initializes from vector of motor structs

  for (auto &motor : motors) motorGroup.insert(std::pair<std::string, Motor>(motor.name, motor));
}


void MotorGroup::add(Motor motor) {
  //adds a motor to the group

  motorGroup.insert(std::pair<std::string, Motor>(motor.name, motor));
}


void MotorGroup::remove(std::string name) {
  //removes a motor from the group

  motorGroup.erase(name);
}


void MotorGroup::enable(std::string name) {
  //enables a motor within the group

  motorGroup.at(name).enabled = true;
}


void MotorGroup::disable(std::string name) {
  //disables a motor within the group

  motorGroup.at(name).enabled = false;
}


void MotorGroup::spin(float voltage, bool useVolt) {
  //spins all motors within the group

  for (auto &motor : motorGroup) {
    if (useVolt && motor.second.enabled) motor.second.motor->spin(vex::fwd, voltage, vex::volt);
    else if (motor.second.enabled) motor.second.motor->spin(vex::fwd, voltage, vex::voltageUnits::mV);
  }
}


void MotorGroup::stop() {
  //stops all motors within the group

  for (auto &motor : motorGroup) if (motor.second.enabled) motor.second.motor->stop();
}


void MotorGroup::setPosition(float pos) {
  //sets all motor encoder positions within the group to a value

  for (auto &motor : motorGroup) if (motor.second.enabled) motor.second.motor->setPosition(pos, vex::deg);
}


void MotorGroup::setStopping(bool coast) {
  //sets the stopping mode of all motors within the group to a type

  for (auto &motor : motorGroup) {
    if (coast && motor.second.enabled) motor.second.motor->setStopping(vex::coast);
    else if (motor.second.enabled) motor.second.motor->setStopping(vex::brake);
  }
}


float MotorGroup::getPosition() {
  //returns the position of the first motor in the group

  return motorGroup.begin()->second.motor->position(vex::deg);
}