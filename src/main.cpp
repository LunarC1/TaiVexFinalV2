#include "opcontrol.h"
#include "auton.h"

vex::competition Competition;


int main() {
  
  imu.calibrate();
  while (imu.isCalibrating()) wait(10, vex::msec);

  brain.Screen.print("hello wrold");

  // cataRotation.resetPosition();
  // catapult.setStopping(vex::coast);
  // catapult.resetPosition();
  
  Competition.autonomous(auton);
  Competition.drivercontrol(opcontrol);
}