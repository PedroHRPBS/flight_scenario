#include "FlightCommand.hpp"

FlightCommand::FlightCommand() {

}

FlightCommand::~FlightCommand() {

}

void FlightCommand::perform(){

    std::cout << "WAITING FLIGHT COMMAND" << std::endl;

    while(_command == flight_command::NULL_TYPE){

    }

}

void FlightCommand::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::FLIGHTCOMMAND){
        FlightCommandMsg* command_msg = (FlightCommandMsg*)t_msg;
        
        _command = static_cast<flight_command>(command_msg->getData());
    }

}