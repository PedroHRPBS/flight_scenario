#include "Wait.hpp"

void Wait::perform(){
    std::cout<< "WAITING TIME" << std::endl;
    Timer _wait_timer;
    _wait_timer.tick();
    while (_wait_timer.tockMilliSeconds()<wait_time_ms)
    {}
    
}

Wait::Wait(){}
Wait::~Wait(){}
