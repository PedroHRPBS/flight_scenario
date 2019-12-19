 #include "ros/ros.h"
#include <iostream>
#include "ROSUnit.hpp"
#include "logger.hpp"
#include "std_logger.hpp"
#include "FlightElement.hpp"
#include "Wait.hpp"
#include "WaitForCondition.hpp"
#include "Arm.hpp"
#include "FlightPipeline.hpp"
#include "SimplePlaneCondition.hpp"
#include "Unarm.hpp"
#include "FlightScenario.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "flight_scenario_manager");
    Logger::assignLogger(new StdLogger());
    //First Pipeline
    Wait wait_1s;
    wait_1s.wait_time_ms=1000;
    Arm arm_motors;
    FlightPipeline default_pipeline;
    default_pipeline.addElement((FlightElement*)&wait_1s);
    default_pipeline.addElement((FlightElement*)&arm_motors);

    SimplePlaneCondition z_cross;
    z_cross.condition_value=1;
    z_cross.condition_met_for_larger=true;
    WaitForCondition z_cross_check;
    z_cross_check.Wait_condition=(Condition*)&z_cross;
    Unarm unarm_motors;
    FlightPipeline safety_pipeline;
    safety_pipeline.addElement((FlightElement*)&z_cross_check);
    safety_pipeline.addElement((FlightElement*)&unarm_motors);
    Logger::getAssignedLogger()->log("FlightScenario main_scenario",LoggerLevel::Info);
    FlightScenario main_scenario;
    main_scenario.AddFlightPipeline(&default_pipeline);
    main_scenario.AddFlightPipeline(&safety_pipeline);
    main_scenario.StartScenario();
    Logger::getAssignedLogger()->log("Main Done",LoggerLevel::Info);
    while(1){

    }
}
