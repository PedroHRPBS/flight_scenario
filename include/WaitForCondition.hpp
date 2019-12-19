#pragma once
#include "FlightElement.hpp"
#include "Condition.hpp"

class WaitForCondition : public FlightElement{
private:
	
public:
	Condtion _wait_condition;
    void perform();
    void receive_msg_data(DataMessage* t_msg);
};
