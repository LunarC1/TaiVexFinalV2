#pragma once

#include "main.h"
#include "motionControl/PID.h"
#include "motionControl/Odometry.h"
#include "motionControl/Odom.h"
#include "motionControl/Motion.h"
#include "motionControl/MotionAlgorithms.h"
#include "components/RotationalArm.h"
#include "utility/Path.h"
#include "PathStorage/testPath.h"
#include "components/PistonGroup.h"


void initialize();
void auton();
void taskHandler(bool driverMode);