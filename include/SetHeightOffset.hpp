#pragma once
#include "FlightElement.hpp"
#include "PositionMsg.hpp"
#include "FloatMsg.hpp"

class SetHeightOffset : public FlightElement{

private:
	float _current_z;

public:

    void perform();

    void receiveMsgData(DataMessage* t_msg);
    
    SetHeightOffset();
    ~SetHeightOffset();
};
