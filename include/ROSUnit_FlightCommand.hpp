#pragma once
#include "common_srv/ROSUnit.hpp"
#include "FlightCommandMsg.hpp"
#include <flight_scenario/Flight_Command.h>

class ROSUnit_FlightCommand :  public ROSUnit{

    private:

        static ROSUnit_FlightCommand* _instance_ptr;
        static FlightCommandMsg _command_msg; 
        ros::ServiceServer _srv_flight_command;
        static bool callbackFlightCommand(flight_scenario::Flight_Command::Request  &req, flight_scenario::Flight_Command::Response &res);
        void receiveMsgData(DataMessage* t_msg);  

    public:
        ROSUnit_FlightCommand(ros::NodeHandle&);
        ~ROSUnit_FlightCommand();
};