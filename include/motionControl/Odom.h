#pragma once

#include "main.h"
#include "components/TrackingWheel.h"
#include "components/Inertial.h"
#include "utility/Misc.h"
#include "utility/Units.h"
#include "utility/Pose.h"


struct OdometrySensors {

    TrackingWheel* parallel;
    TrackingWheel* perpendicular;
    Inertial* inertial;

    OdometrySensors(TrackingWheel* parallel = nullptr, TrackingWheel* perpendicular = nullptr, Inertial* inertial = nullptr)
        : parallel(parallel)
        , perpendicular(perpendicular)
        , inertial(inertial)
    {}
};


struct OdometryOffsets {

    float perpendicularOffset, parallelOffset;

    OdometryOffsets(float perpendicularOffset = 0, float parallelOffset = 0)
        : perpendicularOffset(perpendicularOffset)
        , parallelOffset(parallelOffset)
    {}
};


class TwoWheelInertialOdometry {


    private:

        OdometrySensors &sensors;
        OdometryOffsets &offsets;
        Pose pose;
        std::vector<Pose> positions;
        std::vector<float> angles;
        float lastPerpendicular = 0, lastParallel = 0, lastRotation = 0, unit;

        void updateVelocities();
        float getVelocityX();
        float getVelocityY();


    public:
        
        TwoWheelInertialOdometry(OdometrySensors& sensors, OdometryOffsets& offsets, float unit = inch);

        void updatePosition();
        void setPosition(float x, float y);
        float getUnit();
        Pose getPose();
        float getSpeed();
        float getAngularVelocity();
        float getTranslationDirection();
};