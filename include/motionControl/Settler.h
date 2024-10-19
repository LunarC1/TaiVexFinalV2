#pragma once

#include "main.h"


class Settler {


    private:

        std::function<bool()> condition;


    public:

        Settler(std::function<bool()> condition);

        Settler operator&(Settler& other);
        Settler operator|(Settler& other);

        bool isSettled();
};