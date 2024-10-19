#pragma once

#include "main.h"


#define inch 1
#define foot 1.0/12.0
#define yard 1.0/36.0
#define cm 2.54
#define meter 0.0254


class Units {


  public:

    static float radToDeg(float theta);
    static float degToRad(float theta);
    static float tickToUnit(float num, float wheelDiameter, float unit);
    static float unitToTick(float num, float wheelDiameter, float unit);
    static float unitToUnit(float num, float unit1, float unit2);
};