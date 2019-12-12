#pragma once
#include "FlightElement.hpp"

class Arm : public FlightElement{

public:

    void perform();

    Arm();
    ~Arm();
};