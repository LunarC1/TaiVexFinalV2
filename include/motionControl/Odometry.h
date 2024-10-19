#pragma once

#include "main.h"
#include "utility/Units.h"
#include "utility/Misc.h"


class Odometry {
  

  private:

    float posX = 0, posY = 0, trackWidth = 0, lateralTrackWidth = 0, horizontalTrackWidth = 0, lastLateral = 0, lastHorizontal = 0, lastTheta = 0, unit = inch;
    std::vector<std::pair<float, float>> positions;
    std::vector<float> angles;


  public:

    Odometry(float lateralTrackWidth, float horizontalTrackWidth, float trackWidth)
      : lateralTrackWidth(lateralTrackWidth)
      , horizontalTrackWidth(horizontalTrackWidth)
      , trackWidth(trackWidth)
    {}
    
    void tick();
    void updateDerivative();
    void setPos(float x, float y);
    void setX(float x);
    void setTrackWidths(float lateralTrackWidth, float horizontalTrackWidth);
    void setTrackWidths(std::pair<float, float> offsets);
    void setUnit(float unit);
    float getTrackWidth();
    float getPosX();
    float getPosY();
    float getSpeed();
    float getVelocityTheta();
    float getVelocityX();
    float getVelocityY();
    float getAngularVelocity();
};