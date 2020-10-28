#include "FlightCommand.hpp"

FlightCommand::FlightCommand() {
    _input_port_0 = new InputPort(ports_id::IP_0, this);
    _ports = {_input_port_0};
}

FlightCommand::~FlightCommand() {
}

void FlightCommand::perform(){

    std::cout << "WAITING FLIGHT COMMAND" << std::endl;
    while(_command == flight_command::NULL_TYPE){
    }
    _command = flight_command::NULL_TYPE;
}

void FlightCommand::process(DataMessage* t_msg, Port* t_port) {
    if(t_port->getID() == ports_id::IP_0) {
        FlightCommandMsg* command_msg = (FlightCommandMsg*)t_msg;
        _command = static_cast<flight_command>(command_msg->getData());
    }
}