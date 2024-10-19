#pragma once

#include "main.h"


struct Piston {


  std::string name;
  vex::digital_out* piston;
  bool enabled;

  Piston(std::string name, vex::digital_out* piston, bool enabled = true)
    : name(name)
    , piston(piston)
    , enabled(enabled)
  {}
};


class PistonGroup {


  private:

    std::map<std::string, Piston> pistonGroup;
    

  public:

    PistonGroup(std::vector<Piston> pistons);

    void add(Piston piston);
    void remove(std::string name);
    void enable(std::string name);
    void disable(std::string name);
    void on();
    void off();
    void toggle();
    bool state();
};