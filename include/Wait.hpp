#pragma once
#include "Timer.hpp"

class Wait : public FlightElement{
private:
	
public:
	int wait_time_ms;
    void perform();

    void receive_msg_data(DataMessage* t_msg);
    
    ResetController();
    ~ResetController();
};