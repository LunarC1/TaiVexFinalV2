#include "components/Chassis.h"


float Chassis::getInputVertical(float horizontalInput, float verticalInput) {
    //returns input scaled to a half circle or line depending on driver settings

    if (chassisDriverSettings.useHalfCircleInput) return copysign(Misc::getDistance(horizontalInput, verticalInput, 0, 0), verticalInput);
    return verticalInput;
}


float Chassis::getInputHorizontal(float horizontalInput, float verticalInput) {
    //returns input scaled to a half circle or line depending on driver settings

    if (chassisDriverSettings.useHalfCircleInput) return copysign(Misc::getDistance(horizontalInput, verticalInput, 0, 0), horizontalInput);
    return horizontalInput;
}


float Chassis::scaleExponential(float input) {
    //scales the joystick input exponentially based on driver settings

    return copysign(pow(fabs(input) / (1.0 - chassisDriverSettings.maxOutputThreshold), chassisDriverSettings.outputCurve) / pow(127.0, chassisDriverSettings.outputCurve - 1.0), input);
}


float Chassis::getFStaticFF(float input) {
    //returns the static friction cancellation voltage depending on sign of joystick input

    if (input == 0) return 0;
    return copysign(chassisDriverSettings.fStaticFF, input);
}


float Chassis::inputToOutput(float input, float ff) {
    //converts joystick input to motor output

    return input * (12000.0 - chassisDriverSettings.fStaticFF) / 127.0 + ff;
}


void Chassis::spinTank(float leftInput, float rightInput, bool useVolt, bool reverse) {
    //runs motors of left and right sides of chassis at voltage proportionally to input

    if (!reverse) {
        chassisGeometry.leftMotors->spin(leftInput, useVolt);
        chassisGeometry.rightMotors->spin(rightInput, useVolt);
    }
    else {
        chassisGeometry.leftMotors->spin(-rightInput, useVolt);
        chassisGeometry.rightMotors->spin(-leftInput, useVolt);
    }
}


Tank::Tank(ChassisGeometry chassisGeometry, ChassisDriverSettings chassisDriverSettings) {
    //constructor for tank controls

    this->chassisGeometry = chassisGeometry;
    this->chassisDriverSettings = chassisDriverSettings;
}


void Tank::controllerFeedbackSpin(bool reverse) {
    //spins motors of drivetrain depending on controller input for left and right sides of chassis

    float inputLeft = scaleExponential(getInputVertical(chassisDriverSettings.userController->Axis4.value(), chassisDriverSettings.userController->Axis3.value()));
    float fStaticFFLeft = getFStaticFF(inputLeft);
    float outputLeft = inputToOutput(inputLeft, fStaticFFLeft);
    
    float inputRight = scaleExponential(getInputVertical(chassisDriverSettings.userController->Axis1.value(), chassisDriverSettings.userController->Axis2.value()));
    float fStaticFFRight = getFStaticFF(inputRight);
    float outputRight = inputToOutput(inputRight, fStaticFFRight);
    
    spinTank(outputLeft, outputRight, false, reverse);
}


TwoStickArcade::TwoStickArcade(ChassisGeometry chassisGeometry, ChassisDriverSettings chassisDriverSettings) {
    //constructor for two-stick arcade controls

    this->chassisGeometry = chassisGeometry;
    this->chassisDriverSettings = chassisDriverSettings;
}


void TwoStickArcade::controllerFeedbackSpin(bool reverse) {
    //spins motors of drivetrain depending on controller input for left and right sides of chassis

    float lateralInput = scaleExponential(getInputVertical(chassisDriverSettings.userController->Axis4.value(), chassisDriverSettings.userController->Axis3.value()));
    float turnInput = scaleExponential(getInputHorizontal(chassisDriverSettings.userController->Axis1.value(), chassisDriverSettings.userController->Axis2.value()));
    
    float inputLeft = lateralInput + turnInput;
    float fStaticFFLeft = getFStaticFF(inputLeft);
    float outputLeft = inputToOutput(inputLeft, fStaticFFLeft);

    float inputRight = lateralInput - turnInput;
    float fStaticFFRight = getFStaticFF(inputRight);
    float outputRight = inputToOutput(inputRight, fStaticFFRight);

    spinTank(outputLeft, outputRight, false, reverse);
}


Curvature::Curvature(ChassisGeometry chassisGeometry, ChassisDriverSettings chassisDriverSettings) {
    //constructor for curvature controls
    
    this->chassisGeometry = chassisGeometry;
    this->chassisDriverSettings = chassisDriverSettings;
}


void Curvature::controllerFeedbackSpin(bool reverse) {
    //spins motors of drivetrain depending on controller input for left and right sides of chassis

    float inputPower = scaleExponential(getInputVertical(chassisDriverSettings.userController->Axis4.value(), chassisDriverSettings.userController->Axis3.value()));
    float fStaticFFPower = getFStaticFF(inputPower);
    float outputPower = inputToOutput(inputPower, fStaticFFPower);

    float steering = scaleExponential(getInputHorizontal(chassisDriverSettings.userController->Axis1.value(), chassisDriverSettings.userController->Axis2.value()));
    steering = Misc::clamp(steering * 90.0 / 127.0, -90, 90);

    if (fabs(steering) == 90) spinTank(copysign(outputPower, steering), copysign(outputPower, -steering), false, reverse);
    else {
        std::pair<float, float> outputs = Misc::scaleToRatio(outputPower, 2 + tan(Units::degToRad(steering)) / 2.0, 2 - tan(Units::degToRad(steering)) / 2.0);
        spinTank(outputs.first, outputs.second, false, reverse);
    }
}
