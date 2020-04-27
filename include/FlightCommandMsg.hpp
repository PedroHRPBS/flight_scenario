#pragma once
#include "common_srv/DataMessage.hpp"

class FlightCommandMsg : public DataMessage{

private:
    msg_type _type;
    int _data;

public:

    msg_type getType();
    const int getSize();
    int getData();
    void setFlightCommand(int);

    FlightCommandMsg();
    ~FlightCommandMsg();
};