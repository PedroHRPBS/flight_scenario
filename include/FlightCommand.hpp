#pragma once
#include "FlightElement.hpp"
#include "FlightCommandMsg.hpp"
#include "common_types.hpp"

class FlightCommand : public FlightElement {

private:
    flight_command _command = flight_command::NULL_TYPE;

public:
    void perform();
    void receiveMsgData(DataMessage* t_msg);

    FlightCommand();
    ~FlightCommand();
};