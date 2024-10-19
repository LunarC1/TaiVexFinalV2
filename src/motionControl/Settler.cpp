#include "motionControl/Settler.h"


Settler::Settler(std::function<bool()> condition) {
    //constructor for settler object

    this->condition = condition;
}


Settler Settler::operator&(Settler& other) {
    //combines two settler objects with overloaded & operator

    return Settler([&]() {
        return this->isSettled() && other.isSettled();
    });
}


Settler Settler::operator|(Settler& other) {
    //combines two settler objects with overloaded & operator

    return Settler([&]() {
        return this->isSettled() || other.isSettled();
    });
}


bool Settler::isSettled() {
    //returns the settle state of the settler object

    return condition();
}