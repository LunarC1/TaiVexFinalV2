#pragma once

#include "main.h"


class AsymptoticGains {


  private:

    float i, f, k, p, setpoint;


  public:

    AsymptoticGains(float i, float f, float k, float p);

    void setGain(float setpoint);
    float getGain();
};