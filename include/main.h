#pragma once


#include <vector>
#include <v5.h>
#include <v5_vcs.h>
#include <cstdio>
#include <cmath>
#include <numeric>
#include <memory>
#include <map>


extern vex::controller controller;
extern vex::motor fl;
extern vex::motor fr;
extern vex::motor ml;
extern vex::motor mr;
extern vex::motor bl;
extern vex::motor br;
extern vex::motor intake;
extern vex::motor intake2;
extern vex::motor_group leftDrivetrain;
extern vex::motor_group rightDrivetrain;
extern vex::motor_group drivetrain;
extern vex::brain brain;
extern vex::inertial imu;
extern vex::rotation parallel;
extern vex::rotation sideways;
extern vex::digital_out pisstake;
extern vex::digital_out lift;
extern vex::digital_out mogo;
extern vex::digital_out redirect;
extern vex::digital_out stick;