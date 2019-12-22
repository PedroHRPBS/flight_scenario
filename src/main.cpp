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
#include "Disarm.hpp"
#include "FlightScenario.hpp"
#include "UpdatePIDcontroller.hpp"
#include "UpdatePoseReference.hpp"
#include "ROSUnit_Arm.hpp"
#include "ROSUnit_UpdateController.hpp"
#include "ROSUnit_UpdatePoseReference.hpp"

int main(int argc, char** argv) {
    Logger::assignLogger(new StdLogger());

    //****************ROS stuff********************
    ros::init(argc, argv, "flight_scenario_manager");
    ros::NodeHandle nh;

    FlightElement* arm_motors = new Arm();
    FlightElement* disarm_motors = new Disarm();
    FlightElement* update_controller = new UpdatePIDcontroller();
    FlightElement* takeoff = new UpdatePoseReference();
    FlightElement* land = new UpdatePoseReference();

    ROSUnit* ros_arm_srv = new ROSUnit_Arm(nh);
    ROSUnit* ros_updt_ctr = new ROSUnit_UpdateController(nh);
    ROSUnit* ros_updt_pose_ref = new ROSUnit_UpdatePoseReference(nh);

    arm_motors->add_callback_msg_receiver((msg_receiver*) ros_arm_srv);
    disarm_motors->add_callback_msg_receiver((msg_receiver*) ros_arm_srv);
    update_controller->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    takeoff->add_callback_msg_receiver((msg_receiver*) ros_updt_pose_ref);
    land->add_callback_msg_receiver((msg_receiver*) ros_updt_pose_ref);

    // ((UpdatePIDcontroller*)update_controller)->PIDdata.kp = 0.5;
    // ((UpdatePIDcontroller*)update_controller)->PIDdata.ki = 1;
    // ((UpdatePIDcontroller*)update_controller)->PIDdata.kd = 2;
    // ((UpdatePIDcontroller*)update_controller)->PIDdata.kdd = 0;
    // ((UpdatePIDcontroller*)update_controller)->PIDdata.anti_windup = 0;
    // ((UpdatePIDcontroller*)update_controller)->PIDdata.en_pv_derivation = 0;
    // ((UpdatePIDcontroller*)update_controller)->PIDdata.id = block_id::PID_ROLL;

    // UPDATE BEFORE FLIGHT
    ((UpdatePoseReference*)takeoff)->pose_reference.setPoseMessage(0.0, 0.0, 1.0, 0.0);
    ((UpdatePoseReference*)land)->pose_reference.setPoseMessage(0.0, 0.0, 0.4, 0.0);

    //**********************************************

    //First Pipeline
    Wait wait_1s;
    wait_1s.wait_time_ms=1000;
    FlightPipeline default_pipeline;
    default_pipeline.addElement((FlightElement*)takeoff);
    default_pipeline.addElement((FlightElement*)&wait_1s);
    default_pipeline.addElement((FlightElement*)arm_motors);
    

    SimplePlaneCondition z_cross;
    z_cross.selected_dim=Dimension3D::Z;
    z_cross.condition_value=1;
    z_cross.condition_met_for_larger=true;
    WaitForCondition z_cross_check;
    z_cross_check.Wait_condition=(Condition*)&z_cross;
    FlightPipeline safety_pipeline;
    safety_pipeline.addElement((FlightElement*)&z_cross_check);
    default_pipeline.addElement((FlightElement*)land);
    default_pipeline.addElement((FlightElement*)&wait_1s);
    safety_pipeline.addElement((FlightElement*)disarm_motors);
    Logger::getAssignedLogger()->log("FlightScenario main_scenario",LoggerLevel::Info);
    FlightScenario main_scenario;
    main_scenario.AddFlightPipeline(&default_pipeline);
    main_scenario.AddFlightPipeline(&safety_pipeline);
    main_scenario.StartScenario();
    Logger::getAssignedLogger()->log("Main Done",LoggerLevel::Info);
    while(ros::ok){
        ros::spinOnce();
    }
}
