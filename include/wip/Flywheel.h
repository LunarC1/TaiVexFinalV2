#pragma once

#include "main.h"
#include "motionControl/PID.h"
#include "utility/Filters.h"


class Flywheel {


  private:

    PID &pid;
    AsymptoticGains &KvGains;
    Filter &filter;
    vex::motor* flywheelMotor;
    float target = 0, lastVelocity = 0, Kv = 0;

    void setKv(float target);


  public:

    Flywheel(PID& pid, AsymptoticGains& KvGains, Filter& filter, vex::motor* flywheelMotor)
      : pid(pid)
      , KvGains(KvGains)
      , filter(filter)
      , flywheelMotor(flywheelMotor)
    {}

    float getOutput();
    void setTarget(float target);
};