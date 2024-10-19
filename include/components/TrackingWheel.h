#pragma once

#include "main.h"


struct TrackingWheel {

    vex::rotation &sensor;
    float diameter;

    TrackingWheel(vex::rotation& sensor, float diameter = 2)
        : sensor(sensor)
        , diameter(diameter)
    {}
};