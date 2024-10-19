#pragma once

#include "main.h"
#include "components/Chassis.h"
#include "components/RotationalArm.h"
#include "components/MotorGroup.h"
#include "components/PistonGroup.h"
#include "motionControl/Settler.h"


void opcontrol();

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, vex::msec);                                                             \
  } while (!(condition))