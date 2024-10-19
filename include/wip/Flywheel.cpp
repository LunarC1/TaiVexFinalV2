#include "components/Flywheel.h"


void Flywheel::setKv(float target) {
  //sets the feedforward constant based on target velocity

  Kv = KvGains.getValue(target);
}


float Flywheel::getOutput() {
  //returns one tick of flywheel voltage output given current velocity

  float ffOutput = Kv * target;
  float pidOutput = pid.tick(target - filter.tick(flywheelMotor->velocity(vex::rpm), lastVelocity));
  lastVelocity = flywheelMotor->velocity(vex::rpm);

  return pidOutput + ffOutput;
}


void Flywheel::setTarget(float target) {
  //changes the target velocity of the flywheel

  this->target = target;
  pid.resetIntegral();
  pid.setKp(1);
  setKv(target);
}