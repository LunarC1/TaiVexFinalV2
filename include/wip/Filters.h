#pragma once

#include "main.h"
#include "utility/Misc.h"


class Filter {


  public:

    virtual float tick(float input) {};
    virtual float tick(float input, float prev) {};
};


class StaticEMAFilter : public Filter {


  private:

    float Ka;


  protected:

    std::vector<float> queue;
    int sampleSize;
    float value;

    StaticEMAFilter(int sampleSize)
      : sampleSize(sampleSize)
    {}


  public:

    StaticEMAFilter(int sampleSize, float Ka)
      : sampleSize(sampleSize)
      , Ka(Ka)
    {}

    float tick(float input) override;
};


class DynamicEMAFilter : public StaticEMAFilter {


  private:

    float minKa, maxKa, accelFactor;


  public:

    DynamicEMAFilter(int sampleSize, float minKa, float maxKa, float accelFactor)
      : StaticEMAFilter(sampleSize)
      , minKa(minKa)
      , maxKa(maxKa)
      , accelFactor(accelFactor)
    {}

    float tick(float input, float prev) override;
};


class SlewFilter : public Filter {


  private:

    float maxChange, lastInput = 0;


  public:

    SlewFilter(float maxChange, float max)
      : maxChange(fabs(max) / maxChange)
    {}

    float tick(float input) override;
    void reset();
};