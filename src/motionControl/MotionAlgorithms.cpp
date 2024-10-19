#include "motionControl/MotionAlgorithms.h"


MotionAlgorithms::MotionAlgorithms(Tank &chassis, float unit)
    //constructor for motion algorithms object

    : chassis(chassis)
    , unit(unit)
{}


void MotionAlgorithms::set1DSensors(TrackingWheel* parallel, Inertial* inertial) {
    //sets sensors for one-dimensional feedback movements

    this->parallel = parallel;
    this->inertial = inertial;
}


void MotionAlgorithms::set1DPIDControllers(OneDimPIDControllers* PID1D) {
    //sets PID controllers for one-dimensional PID movements

    this->PID1D = PID1D;
}


void MotionAlgorithms::set2DPIDControllers(TwoDimPIDControllers* PID2D) {
    //sets PID controllers for two-dimensional PID movements

    this->PID2D = PID2D;
}


void MotionAlgorithms::setOdometry(TwoWheelInertialOdometry* odometry) {
    //sets odometry for two-dimensional feedback movements

    this->odometry = odometry;
}


void MotionAlgorithms::move1D(float distance, float timeout, move1DParams& params) {
    //moves forward for target distance

    uint64_t startTime = vex::timer::systemHighResolution();

    PID1D->lateral.reset(distance);
    PID1D->lateral.setKp(distance);

    PID1D->correctionAngular.reset(Misc::angleError(headingTarget, inertial->getHeading()));
    PID1D->correctionAngular.setKp(Misc::angleError(headingTarget, inertial->getHeading()));

    distance += Units::tickToUnit(parallel->sensor.position(vex::deg), parallel->diameter, unit);

    do {

        uint64_t timestamp = vex::timer::systemHighResolution();

        float lateralError = distance - Units::tickToUnit(parallel->sensor.position(vex::deg), parallel->diameter, unit);
        float angularError = Misc::angleError(headingTarget, inertial->getHeading());
        
        float lateralOutput = Misc::clamp(PID1D->lateral.tick(lateralError), -params.maxSpeed * 12000.0, params.maxSpeed * 12000.0);
        float angularOutput = Misc::clamp(PID1D->correctionAngular.tick(angularError), -params.maxSpeed * 12000.0, params.maxSpeed * 12000.0);
        
        std::pair<float, float> outputs = Misc::reduceRatio(params.maxSpeed * 12000.0, lateralOutput + angularOutput, lateralOutput - angularOutput);
        chassis.spinTank(outputs.first, outputs.second);

        wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
    }
    while (( vex::timer::systemHighResolution() < startTime + timeout * 1000.0
          && params.exitConditions.isSettled() == false));
}