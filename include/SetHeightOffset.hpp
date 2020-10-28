#pragma once
#include "FlightElement.hpp"
#include "PositionMsg.hpp"
#include "common_srv/FloatMsg.hpp"

class SetHeightOffset : public FlightElement{

private:
    Port* _input_port_0;
    Port* _output_port_0;
	float _current_z;

public:

    enum ports_id {IP_0, OP_0};
    void process(DataMessage* t_msg, Port* t_port);
    void perform();
    
    SetHeightOffset();
    ~SetHeightOffset();
};
