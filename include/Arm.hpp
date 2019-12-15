#pragma once
#include "FlightElement.hpp"

class Arm : public FlightElement{

public:

    void perform();
    void receive_msg_data(DataMessage* t_msg);
    
    Arm();
    ~Arm();
};