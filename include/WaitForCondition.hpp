#pragma once
#include "FlightElement.hpp"
#include "Condition.hpp"

class WaitForCondition : public FlightElement{
private:
	Condition* m_wait_condition;
public:
    void perform();
    void receiveMsgData(DataMessage* t_msg);
    WaitForCondition(Condition*);
};
