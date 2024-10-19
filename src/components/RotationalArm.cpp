#include "components/RotationalArm.h"


Lift::Lift(MotorGroup* motorGroup)
  //constructor for rotational arm project

  : motorGroup(motorGroup)
{}

    
Lift::Lift(MotorGroup* motorGroup, PID* pid)
  //constructor for rotational arm project

  : motorGroup(motorGroup)
  , pid(pid)
{}

    
Lift::Lift(MotorGroup* motorGroup, vex::rotation* armRotation)
  //constructor for rotational arm project

  : motorGroup(motorGroup)
  , armRotation(armRotation)
{}
    

Lift::Lift(MotorGroup* motorGroup, vex::rotation* armRotation, PID* pid)
  //constructor for rotational arm project

  : motorGroup(motorGroup)
  , armRotation(armRotation)
  , pid(pid)
{}


void Lift::reset() {
  //spins the arm to a user-defined target
  
  Lift::updateStatus();

  float output;
  float armPosition = (armRotation != nullptr) ? armRotation->position(vex::deg) : fmod(fabs(motorGroup->getPosition()), 180);

  if (pid != nullptr) {
    output = pid->tick(resetSetpoint - armPosition);
    output = Misc::clamp(output, -power * 12, power * 12);
  }
  else {
    if (!reachedSetpoint) output = (resetSetpoint > armPosition) ? power * 12 : -power * 12;
    else output = 0;
  }

  motorGroup->spin(output, true);
}


void Lift::updateStatus() {
  //state machine for arm

  bool currentCrossingStatus = (armRotation != nullptr) ? armRotation->position(vex::deg) > resetSetpoint : fmod(fabs(motorGroup->getPosition()), 180) > resetSetpoint;
  if (currentCrossingStatus != lastCrossingStatus) reachedSetpoint = true;
  lastCrossingStatus = currentCrossingStatus;
}


void Lift::setResetSetpoint(float resetSetpoint) {
  //sets the arm setpoint to a new value

  if (this->resetSetpoint != resetSetpoint) {
    this->resetSetpoint = resetSetpoint;
    reachedSetpoint = false;
    lastCrossingStatus = (armRotation != nullptr) ? armRotation->position(vex::deg) > resetSetpoint : fmod(fabs(motorGroup->getPosition()), 180) > resetSetpoint;
  }
}


void Lift::setPower(float power) {
  //sets the arm power to a new value

  this->power = power;
}


Cata::Cata(MotorGroup* motorGroup)
  //constructor for catapult object

  : Lift(motorGroup)
{}


Cata::Cata(MotorGroup* motorGroup, PID* pid)
  //constructor for catapult object

  : Lift(motorGroup, pid)
{}


Cata::Cata(MotorGroup* motorGroup, vex::rotation* cataRotation)
  //constructor for catapult object

  : Lift(motorGroup, cataRotation)
{}


Cata::Cata(MotorGroup* motorGroup, vex::rotation* cataRotation, PID* pid)
  //constructor for catapult object

  : Lift(motorGroup, cataRotation, pid)
{}


void Cata::reset() {
  //spins the catapult to a pre-defined target

  Cata::updateStatus();
  
  float cataPosition = (armRotation != nullptr) ? fabs(armRotation->position(vex::deg)) : fmod(fabs(motorGroup->getPosition()), 180);

  float output;
  if (pid != nullptr) output = (currentlyShooting) ? power * 12 : Misc::clamp(pid->tick(resetSetpoint - cataPosition), 0, power * 12);
  else output = (currentlyShooting || !reachedSetpoint) ? power * 12 : 0;

  motorGroup->spin(output, true);
}


void Cata::release() {
  //spins the catapult to release the triball

  Cata::updateStatus();

  motorGroup->spin(power * 12, true);
}


void Cata::shoot(int amt, int timeout) {
  //shoots the triball a user-specified amount of times

  int initialCount = shotCounter;

  uint64_t startTime = vex::timer::systemHighResolution();

  do {

    uint64_t timestamp = vex::timer::systemHighResolution();

    Cata::release();

    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
  while (shotCounter < initialCount + amt
      && (vex::timer::systemHighResolution() - startTime) / 1000.0 < timeout); 

  motorGroup->stop();
}


void Cata::updateStatus() {
  //state machine for the catapult

  float cataPosition = (armRotation != nullptr) ? fabs(armRotation->position(vex::deg)) : fmod(fabs(motorGroup->getPosition()), 180);

  if (currentlyShooting && fabs(cataPosition) < lowerBound) {
    shotCounter++;
    currentlyShooting = false;
    reachedSetpoint = false;
  }
  if (fabs(cataPosition) > upperBound) currentlyShooting = true;
  if (fabs(cataPosition) > resetSetpoint) reachedSetpoint = true;
}


int Cata::getShotCounter() {
  //returns the amount of times that the catapult has shot

  return shotCounter;
}


void Cata::setShotCounterBounds(float lowerBound, float upperBound) {
  //sets the lower and upper parameters for the catapult shot counter

  this->lowerBound = lowerBound;
  this->upperBound = upperBound;
}


DistCata::DistCata(MotorGroup* motorGroup, vex::distance* cataDistance)
  //constructor for distance sensor-type catapult object

  : Cata(motorGroup)
  , cataDistance(cataDistance)
{}


DistCata::DistCata(MotorGroup* motorGroup, PID* pid, vex::distance* cataDistance)
  //constructor for distance sensor-type catapult object

  : Cata(motorGroup, pid)
  , cataDistance(cataDistance)
{}


DistCata::DistCata(MotorGroup* motorGroup, vex::rotation* cataRotation, vex::distance* cataDistance)
  //constructor for distance sensor-type catapult object
  
  : Cata(motorGroup, cataRotation)
  , cataDistance(cataDistance)
{}


DistCata::DistCata(MotorGroup* motorGroup, vex::rotation* cataRotation, PID* pid, vex::distance* cataDistance)
  //constructor for distance sensor-type catapult object
  
  : Cata(motorGroup, cataRotation, pid)
  , cataDistance(cataDistance)
{}


void DistCata::reset() {
  //spins the catapult to a pre-defined target

  DistCata::updateStatus();
  firstShot = true;

  float cataPosition = (armRotation != nullptr) ? fabs(armRotation->position(vex::deg)) : fmod(fabs(motorGroup->getPosition()), 180);

  float output;
  if (pid != nullptr) output = (currentlyShooting) ? power * 12 : Misc::clamp(pid->tick(resetSetpoint - cataPosition), 0, power * 12);
  else output = (currentlyShooting || !reachedSetpoint) ? power * 12 : 0;

  motorGroup->spin(output, true);
}


void DistCata::release() {
  //spins the catapult to release the triball

  bool withinRange = cataDistance->objectDistance(vex::distanceUnits::in) < distanceRange;
  if (withinRange || (!releaseOnlyInRange && !firstShot) || triballDetected) {
    if (withinRange && currentlyShooting) triballDetected = true;
    updateStatus();
    motorGroup->spin(power * 12, true);
  }
  else reset();
}


void DistCata::shoot(int amt, int timeout) {
  //shoots the triball a user-specified amount of times

  int initialCount = shotCounter;

  uint64_t startTime = vex::timer::systemHighResolution();

  do {

    uint64_t timestamp = vex::timer::systemHighResolution();

    DistCata::release();

    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }
  while (shotCounter < initialCount + amt
      && (vex::timer::systemHighResolution() - startTime) / 1000.0 < timeout); 

  motorGroup->stop();
}


void DistCata::updateStatus() {
  //state machine for the catapult

  float cataPosition = (armRotation != nullptr) ? fabs(armRotation->position(vex::deg)) : fmod(fabs(motorGroup->getPosition()), 180);

  if (currentlyShooting && fabs(cataPosition) < lowerBound) {
    shotCounter++;//if (triballDetected) shotCounter++;
    triballDetected = false;
    currentlyShooting = false;
    reachedSetpoint = false;
    firstShot = false;
  }
  if (fabs(cataPosition) > upperBound) currentlyShooting = true;
  if (fabs(cataPosition) > resetSetpoint) reachedSetpoint = true;
}


void DistCata::setDistanceParams(float distanceRange, bool releaseOnlyInRange) {
  //sets the settings for the distance sensor for the catapult

  this->distanceRange = distanceRange;
  this->releaseOnlyInRange = releaseOnlyInRange;
}