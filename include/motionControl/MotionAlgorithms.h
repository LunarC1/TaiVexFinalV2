#pragma once

#include "main.h"
#include "components/Chassis.h"
#include "components/TrackingWheel.h"
#include "components/Inertial.h"
#include "motionControl/PID.h"
#include "motionControl/Odom.h"
#include "motionControl/Settler.h"
#include "utility/Misc.h"
#include "utility/Pose.h"
#include "utility/Path.h"
#include "utility/Units.h"


struct OneDimPIDControllers {

    PID &lateral, &rotationAngular, &correctionAngular;

    OneDimPIDControllers(PID& lateral, PID& rotationAngular, PID& correctionAngular)
        : lateral(lateral)
        , rotationAngular(rotationAngular)
        , correctionAngular(correctionAngular)
    {}
};


struct TwoDimPIDControllers {

    PID &lateral, &angular;

    TwoDimPIDControllers(PID& lateral, PID& angular) 
        : lateral(lateral)
        , angular(angular)
    {}
};


struct move1DParams {

    float minSpeed = 0;
    float maxSpeed = 1;
    bool reverse = true;
    Settler& exitConditions;
};


class MotionAlgorithms {

    
    private:

        Tank &chassis;
        TrackingWheel* parallel = nullptr;
        Inertial* inertial = nullptr;
        OneDimPIDControllers* PID1D = nullptr;
        TwoDimPIDControllers* PID2D = nullptr;
        TwoWheelInertialOdometry* odometry = nullptr;
        float unit, headingTarget = 0;


    public:

        MotionAlgorithms(Tank& chassis, float unit = inch);

        void set1DSensors(TrackingWheel* parallel, Inertial* inertial);
        void set1DPIDControllers(OneDimPIDControllers* PID1D);
        void set2DPIDControllers(TwoDimPIDControllers* PID2D);
        void setOdometry(TwoWheelInertialOdometry* odometry);

        void move1D(float distance, float timeout, move1DParams& params);
};

