#pragma once

#include "main.h"
#include "utility/Misc.h"
#include "components/MotorGroup.h"


struct ChassisGeometry {
    
        
    MotorGroup* leftMotors;
    MotorGroup* rightMotors;
    float trackWidth;

    ChassisGeometry(MotorGroup* leftMotors = nullptr, MotorGroup* rightMotors = nullptr, float trackWidth = -1)
        : leftMotors(leftMotors)
        , rightMotors(rightMotors)
        , trackWidth(trackWidth)
    {}
};


struct ChassisDriverSettings {


    vex::controller* userController;
    float outputCurve;
    float maxOutputThreshold;
    float fStaticFF;
    bool useHalfCircleInput;

    ChassisDriverSettings(vex::controller* userController = nullptr, float outputCurve = 1, float maxOutputThreshold = 0, float fStaticFF = 0, bool useHalfCircleInput = false)
        : userController(userController)
        , outputCurve(outputCurve)
        , maxOutputThreshold(maxOutputThreshold)
        , fStaticFF(fStaticFF)
        , useHalfCircleInput(useHalfCircleInput)
    {}
};


class Chassis {


    protected:

        ChassisGeometry chassisGeometry;
        ChassisDriverSettings chassisDriverSettings;

        float getInputVertical(float horizontalInput, float verticalInput);
        float getInputHorizontal(float horizontaInput, float verticalInput);
        float scaleExponential(float input);
        float getFStaticFF(float input);
        float inputToOutput(float input, float ff); 
    

    public:

        virtual void controllerFeedbackSpin(bool reverse = false) = 0;
        void spinTank(float leftInput, float rightInput, bool useVolt = false, bool reverse = false);
};


class Tank : public Chassis {


    public:

        Tank(ChassisGeometry chassisGeometry, ChassisDriverSettings chassisDriverSettings);

        void controllerFeedbackSpin(bool reverse = false) override;
};


class TwoStickArcade : public Chassis {


    public:

        TwoStickArcade(ChassisGeometry chassisGeometry, ChassisDriverSettings chassisDriverSettings);

        void controllerFeedbackSpin(bool reverse = false) override;
};


class Curvature : public Chassis {
        

    public:

        Curvature(ChassisGeometry chassisGeometry, ChassisDriverSettings chassisDriverSettings);

        void controllerFeedbackSpin(bool reverse = false) override;
};