#pragma once

#include "main.h"
#include "utility/AsymptoticGains.h"


class PID {


  private:

    AsymptoticGains &Kp;
    float Ki, Kd, integralDeadband, error, totalError = 0, lastError = 0, derivative;
    bool integralSignReset;


  public:

    PID(AsymptoticGains& Kp, float Ki = 0, float Kd = 0, float integralDeadband = 0, bool integralSignReset = false);

    float tick(float error);
    void reset(float initialError);
    void setKp(float setpoint);
    float getError();
};