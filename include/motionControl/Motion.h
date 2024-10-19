#pragma once

#include "main.h"
#include "motionControl/PID.h"
#include "utility/Misc.h"
#include "motionControl/Odometry.h"
#include "utility/Pose.h"
#include "motionControl/Exit.h"
#include "utility/Path.h"


class Motion {


  private:

    PID &lateral, &turn, &headingCorrect;
    Odometry &odometry;
    float headingTarget = 0, unit = inch, pursuitVeloConst = 0, pursuitCurvConst = 0, pursuitCTEConst = 0, minLookahead = 1, maxLookahead = 1, radiusLookahead = 5, distTravelled = -2;
    

  public:

    Motion(PID& lateral, PID& turn, PID& headingCorrect, Odometry& odometry)
      : lateral(lateral)
      , turn(turn)
      , headingCorrect(headingCorrect)
      , odometry(odometry)
    {}

    void setUnit(float unit);
    void setPursuitSettings(float minLookahead, float maxLookahead, float pursuitVeloConst, float pursuitCurvConst, float pursuitCTEConst, float radiusLookahead);

    void moveDist(float target, float maxSpeed, float timeout);

    void push(float speed, float timeout);
    void push(float coord, bool isY, float speed, float timeout);
    void crossBarrier();
    
    void turnHeading(float target, int sides, float minSpeed, float maxSpeed, float timeout);
    void turnPoint(float x, float y, int sides, float minSpeed, float maxSpeed, float timeout);
    void turnPointOneIter(float x, float y, int sides, float minSpeed, float maxSpeed);
    
    void movePoint(float x, float y, float minSpeed, float maxSpeed, float timeout, bool settle = true, bool endIntake = false);
    void movePose(float x, float y, float theta, float dLead, float gLead, float minSpeed, float maxSpeed, float timeout, float chasePower = -1, bool settle = true, bool endIntake = false);

    void followPursuit(Path path, float timeout, float chasePower = -1, bool reverse = false, bool endIntake = false);
    void followStanley(Path path, float timeout, float k, float chasePower = -1, bool reverse = false, bool endIntake = false);
    void followAPS(Path path, float timeout, float maxSpeed = 1, float chasePower = -1, bool reverse = false, bool endIntake = false);

    void asdf(float targetAngle, float targetSpeed);
};