#include "components/Inertial.h"

Inertial::Inertial(vex::inertial& inertial, float k)
    //constructor for inertial sensor object

    : inertial(inertial)
    , k(k)
{}


float Inertial::getHeading(bool rad) {
    //returns inertial heading constrained between [-180, 180] degrees or [-pi, pi] radians

    if (rad) return Misc::reduceAngle(inertial.rotation(vex::deg) * k) * M_PI / 180.0;
    return Misc::reduceAngle(inertial.rotation(vex::deg) * k);
}


float Inertial::getRotation(bool rad) {
    //returns total inertial rotation in either degrees or radians

    if (rad) return inertial.rotation(vex::deg) * k * M_PI / 180.0;
    return inertial.rotation(vex::deg) * k;
}