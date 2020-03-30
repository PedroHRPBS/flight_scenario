#pragma once
#include "FlightElement.hpp"
#include "Timer.hpp"

class Wait : public FlightElement{
private:
	
public:
	int wait_time_ms;
    void perform();

    void receiveMsgData(DataMessage* t_msg);
    
    Wait();
    ~Wait();
};