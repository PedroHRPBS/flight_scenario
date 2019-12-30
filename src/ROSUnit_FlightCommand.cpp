#include "ROSUnit_FlightCommand.hpp"
ROSUnit_FlightCommand* ROSUnit_FlightCommand::_instance_ptr = NULL;
FlightCommandMsg ROSUnit_FlightCommand::_command_msg;

ROSUnit_FlightCommand::ROSUnit_FlightCommand(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_flight_command = t_main_handler.advertiseService("flight_command", callbackFlightCommand);
    _instance_ptr = this;
}   

ROSUnit_FlightCommand::~ROSUnit_FlightCommand() {

}

void ROSUnit_FlightCommand::receive_msg_data(DataMessage* t_msg){


}

bool ROSUnit_FlightCommand::callbackFlightCommand(flight_scenario::Flight_Command::Request &req, flight_scenario::Flight_Command::Response &res){

    int data;
    data = req.flight_command_id;

    _command_msg.setFlightCommand(data);
    _instance_ptr->emit_message((DataMessage*) &_command_msg);
    
    return true;
}