#include "motionControl/Exit.h"


bool Exit::crossed = false;


bool Exit::error(float error, float minError) {

  return fabs(error) > minError;
}


bool Exit::velo(float velo, float minVelo) {

  return velo > minVelo;
}


bool Exit::hc(Pose pose, Pose target, float theta, float tolerance) {

  return (pose.y - target.y) * -cos(Units::degToRad(theta)) >= sin(Units::degToRad(theta)) * (pose.x - target.x) + tolerance;
}


bool Exit::time(int time, int timeout) {

  return time < timeout;
}


bool Exit::intaked(vex::motor intake) {
  
  if (!crossed && intake.velocity(vex::rpm) > 675) crossed = true;
  if (intake.velocity(vex::rpm) < 600 && crossed) {
    crossed = false;
    return false;
  }
  return true;
}