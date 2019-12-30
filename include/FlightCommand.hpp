#pragma once
#include "FlightElement.hpp"
#include "FlightCommandMsg.hpp"

class FlightCommand : public FlightElement {

private:
    flight_command _command = flight_command::NULL_TYPE;

public:
    void perform();
    void receive_msg_data(DataMessage* t_msg);

    FlightCommand();
    ~FlightCommand();
};