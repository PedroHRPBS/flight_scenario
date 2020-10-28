#pragma once
#include "FlightElement.hpp"
#include "ArmDataMessage.hpp"
class Arm : public FlightElement {

private:
    Port* _output_port_0;

public:
    enum ports_id {OP_0};
    void process(DataMessage* t_msg, Port* t_port) {};
    void perform();
    Arm();
    ~Arm();
};