#pragma once

#include "main.h"
#include "utility/Misc.h"
#include "utility/Pose.h"
#include "motionControl/Odometry.h"


class Exit  {

  
  private:

    static bool crossed;


  public:

    static bool error(float minError, float error);
    static bool velo(float velo, float minVelo);
    static bool hc(Pose pose, Pose target, float theta, float tolerance);
    static bool time(int time, int timeout);
    static bool intaked(vex::motor intake);
};