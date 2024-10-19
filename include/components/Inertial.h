#pragma once

#include "main.h"
#include "utility/Misc.h"

class Inertial {


    private:

        vex::inertial &inertial;
        float k;


    public:

        Inertial(vex::inertial& inertial, float k = 1);

        float getHeading(bool rad = false);
        float getRotation(bool rad = false);
};