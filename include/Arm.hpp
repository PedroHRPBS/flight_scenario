#pragma once
#include "FlightElement.hpp"
#include "ArmDataMessage.hpp"
class Arm : public FlightElement{

public:

    void perform();
    void receiveMsgData(DataMessage* t_msg);
    
    Arm();
    ~Arm();
};