#pragma once
#include "FlightElement.hpp"
#include "FlightCommandMsg.hpp"
#include "common_types.hpp"

class FlightCommand : public FlightElement {

private:
    Port* _input_port_0;
    flight_command _command = flight_command::NULL_TYPE;

public:

    enum ports_id {IP_0};
    void process(DataMessage* t_msg, Port* t_port);
    void perform();

    FlightCommand();
    ~FlightCommand();
};