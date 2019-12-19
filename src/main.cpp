#include <iostream>
#include "FlightPipeline.hpp"
#include "Arm.hpp"
#include "UpdatePIDcontroller.hpp"
#include "UpdateReference.hpp"
#include "ResetController.hpp"
#include "Disarm.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "flight_scenario_node");

    ros::NodeHandle nh;
    ros::Rate rate(300);

    std::cout << "Hello Easy C++ project!" << std::endl;

    FlightPipeline* myFirstFlight = new FlightPipeline();

    FlightElement* arm = new Arm(nh);
    FlightElement* disarm = new Disarm(nh);
    FlightElement* updatecontroller = new UpdatePIDcontroller(nh);
    FlightElement* resetcontroller = new ResetController(nh);
    FlightElement* updatereference = new UpdateReference(nh);

    //myFirstFlight->addElement(updatecontroller);
    myFirstFlight->addElement(updatereference);
    myFirstFlight->addElement(arm);

    myFirstFlight->execute();

    // ros::Duration(5).sleep();

    // myFirstFlight->addElement(disarm);

    // myFirstFlight->execute();

    while(ros::ok){
        ros::spinOnce();
        rate.sleep();
    }
}