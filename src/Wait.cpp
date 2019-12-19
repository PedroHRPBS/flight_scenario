#include "Wait.hpp"

void Wait::perform(){
    Timer _wait_timer;
    _wait_timer.tick();
    while (_wait_timer.tockMilliSeconds()<wait_time_ms)
    {}
    
}

void Wait::receive_msg_data(DataMessage* t_msg){}

Wait::Wait(ros::NodeHandle& t_main_handler) : FlightElement(t_main_handler){}
Wait::~Wait(){}
