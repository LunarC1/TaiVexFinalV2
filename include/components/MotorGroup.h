#pragma once

#include "main.h"


struct Motor {


  std::string name;
  vex::motor* motor;
  bool enabled;

  Motor(std::string name, vex::motor* motor, bool enabled = true)
    : name(name)
    , motor(motor)
    , enabled(enabled)
  {}
};


class MotorGroup {


  private:

    std::map<std::string, Motor> motorGroup;
    

  public:

    MotorGroup(std::vector<Motor> motors);

    void add(Motor motor);
    void remove(std::string name);
    void enable(std::string name);
    void disable(std::string name);
    void spin(float voltage, bool useVolt = true);
    void stop();
    void setPosition(float pos);
    void setStopping(bool coast);
    float getPosition();
};