#pragma once

#include "main.h"
#include "utility/Units.h"
#include "utility/Pose.h"


class Pose;
class Misc {


  public:

    static float getHeading(bool reverse);
    static void runDrivetrain(float left, float right, bool reverse);
    static float clamp(float input, float min, float max);
    static float getDistance(float x1, float y1, float x2, float y2);
    static float reduceAngle(float theta);
    static float angleError(float target, float theta);
    static std::pair<float, float> scaleToRatio(float factor, float num1, float num2);
    static std::pair<float, float> reduceRatio(float max, float num1, float num2);
    static float getCurvature(Pose p1, Pose p2);
    static float getCurvature(Pose p1, Pose p2, float heading);
    static float getCurvature(Pose p1, Pose p2, Pose p3);
    static float getRadius(Pose p1, Pose p2);
    static float getRadius(Pose p1, Pose p2, Pose p3);
    static float circleIntersect(Pose linep1, Pose linep2, Pose center, float radius);
    static Pose lerp(Pose p1, Pose p2, float t);
};