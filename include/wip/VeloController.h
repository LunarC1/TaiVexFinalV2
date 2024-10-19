#pragma once

#include "main.h"
#include "motionControl/PID.h"
#include "utility/Filters.h"


class VeloController {


  private:

    std::shared_ptr<PID> pid;
    std::shared_ptr<AsymptoticGains> ff;
    std::shared_ptr<Filter> filter;
    float Kv = 0, Ka = 0;


  public:

    VeloController(std::shared_ptr<PID> pid, std::shared_ptr<AsymptoticGains> ff, float Ka, std::shared_ptr<Filter> filter)
      : pid(pid)
      , ff(ff)
      , Ka(Ka)
      , filter(filter)
    {}

    VeloController(std::shared_ptr<PID> pid, std::shared_ptr<AsymptoticGains> ff, float Ka)
      : pid(pid)
      , ff(ff)
      , Ka(Ka)
    {}

    VeloController(std::shared_ptr<AsymptoticGains> ff, float Ka, std::shared_ptr<Filter> filter)
      : ff(ff)
      , Ka(Ka)
    {}

    float getOutput(float velocity);
    void setKv(float target);
    void setTarget(float target);
};