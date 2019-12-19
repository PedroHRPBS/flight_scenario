#pragma once
#include "FlightElement.hpp"
#include "Condition.hpp"

class WaitForCondition : public FlightElement{
private:
	
public:
	Condition* Wait_condition;
    void perform();
    void receive_msg_data(DataMessage* t_msg);
};
