#include "ROSUnit_FlightCommand.hpp"
ROSUnit_FlightCommand* ROSUnit_FlightCommand::_instance_ptr = NULL;
FlightCommandMsg ROSUnit_FlightCommand::_command_msg;
Port* ROSUnit_FlightCommand::_output_port_0 = new OutputPort(ports_id::OP_0, NULL);

ROSUnit_FlightCommand::ROSUnit_FlightCommand(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_flight_command = t_main_handler.advertiseService("flight_command", callbackFlightCommand);
    _instance_ptr = this;
    _ports = {_output_port_0};
}   

ROSUnit_FlightCommand::~ROSUnit_FlightCommand() {

}

bool ROSUnit_FlightCommand::callbackFlightCommand(flight_scenario::Flight_Command::Request &req, flight_scenario::Flight_Command::Response &res){

    int data;
    data = req.flight_command_id;

    _command_msg.setFlightCommand(data);
    _output_port_0->receiveMsgData((DataMessage*) &_command_msg);
    
    return true;
}