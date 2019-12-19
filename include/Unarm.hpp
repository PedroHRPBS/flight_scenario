#pragma once
#include "FlightElement.hpp"
#include "ArmDataMessage.hpp"

class Unarm : public FlightElement{

public:

    void perform();
    void receive_msg_data(DataMessage* t_msg);
    
    Unarm();
    ~Unarm();
};