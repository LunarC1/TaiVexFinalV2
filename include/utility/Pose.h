#pragma once

#include "main.h"
#include "utility/Misc.h"
#include "utility/Units.h"


class Pose {


  public:

    float x, y, theta;

    Pose(float x = 0, float y = 0, float theta = 0);

    Pose operator+(const Pose& other);
    Pose operator-(const Pose& other);
    Pose operator*(const Pose& other);
    Pose operator/(const Pose& other);
    Pose operator=(const Pose& other);
    Pose operator+(const float& other);
    Pose operator-(const float& other);
    Pose operator*(const float& other);
    Pose operator/(const float& other);

    float dotProduct(Pose other);
    float distance(Pose other);
    float face(Pose other, bool rad = false);
    float angle(Pose other, bool rad = false);
    float parallel(Pose other);
    float parallel(float target);
    void set(float x, float y, float theta);
};