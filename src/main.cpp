#include <iostream>
#include "FlightPipeline.hpp"
#include "FlightElement.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "flight_scenario_node");

    ros::NodeHandle nh;
    ros::Rate rate(300);

    std::cout << "Hello Easy C++ project!" << std::endl;

    FlightPipeline* myFirstFlight = new FlightPipeline();


    while(ros::ok){
        ros::spinOnce();
        rate.sleep();
    }
}