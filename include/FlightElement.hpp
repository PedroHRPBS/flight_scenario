#pragma once

class FlightElement {

public:

    virtual void perform() = 0;

    FlightElement();
    ~FlightElement();
};