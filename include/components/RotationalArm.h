#pragma once

#include "main.h"
#include "components/MotorGroup.h"
#include "motionControl/PID.h"
#include "utility/Misc.h"


class RotationalArm {


  private:

    virtual void updateStatus() = 0;


  public:

    virtual void reset() = 0;
    virtual void release() {};
    virtual void shoot(int amt, int timeout) {};
};


class Lift : public RotationalArm {


  private:

    void updateStatus() override;


  protected:

    MotorGroup* motorGroup = nullptr;
    vex::rotation* armRotation = nullptr;
    PID* pid = nullptr;
    float power = 0, resetSetpoint = 0;
    bool reachedSetpoint = false, lastCrossingStatus;


  public:

    Lift(MotorGroup* motorGroup);
    Lift(MotorGroup* motorGroup, PID* pid);   
    Lift(MotorGroup* motorGroup, vex::rotation* armRotation);  
    Lift(MotorGroup* motorGroup, vex::rotation* armRotation, PID* pid);
    
    void reset() override;
    void setResetSetpoint(float resetSetpoint);
    void setPower(float power);
};


class Cata : public Lift {


  private:

    void updateStatus() override;


  protected:

    int shotCounter = 0;
    float lowerBound = 0, upperBound = 0;
    bool currentlyShooting = false;


  public:

    Cata(MotorGroup* motorGroup);
    Cata(MotorGroup* motorGroup, PID* pid);
    Cata(MotorGroup* motorGroup, vex::rotation* cataRotation);
    Cata(MotorGroup* motorGroup, vex::rotation* cataRotation, PID* pid);
    
    void reset() override;
    void release() override;
    void shoot(int amt, int timeout) override;
    int getShotCounter();
    void setShotCounterBounds(float lowerBound, float upperBound);
};


class DistCata : public Cata {


  private:

    vex::distance* cataDistance = nullptr;
    float distanceRange = 0;
    bool releaseOnlyInRange = false, firstShot = true, triballDetected = false;

    void updateStatus() override;


  public:

    DistCata(MotorGroup* motorGroup, vex::distance* cataDistance);
    DistCata(MotorGroup* motorGroup, PID* pid, vex::distance* cataDistance);
    DistCata(MotorGroup* motorGroup, vex::rotation* cataRotation, vex::distance* cataDistance);
    DistCata(MotorGroup* motorGroup, vex::rotation* cataRotation, PID* pid, vex::distance* cataDistance);
    
    void reset() override;
    void release() override;
    void shoot(int amt, int timeout) override;
    void setDistanceParams(float distanceRange, bool releaseOnlyInRange);
};