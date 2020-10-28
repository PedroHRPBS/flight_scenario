#pragma once
#include "FlightElement.hpp"
#include "Timer.hpp"

class Wait : public FlightElement{
private:
	
public:
	int wait_time_ms;
    void perform();

    void process(DataMessage* t_msg, Port* t_port) {};
    
    Wait();
    ~Wait();
};