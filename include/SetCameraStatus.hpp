#pragma once
#include "FlightElement.hpp"
#include "common_srv/IntegerMsg.hpp"

class SetCameraStatus : public FlightElement{

private:
	int _current_status;

public:

    void perform();

    void receiveMsgData(DataMessage* t_msg);
    
    SetCameraStatus(int);
    ~SetCameraStatus();
};
