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
#include "ResetController.hpp"
#include "ROSUnit_Arm.hpp"
#include "ROSUnit_UpdateController.hpp"
#include "ROSUnit_UpdatePoseReference.hpp"
#include "ROSUnit_PositionSubscriber.hpp"
#include "ROSUnit_ResetController.hpp"

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
    ResetController* reset_z = new ResetController();
    reset_z->target_block = block_id::PID_Z;

    ROSUnit* ros_arm_srv = new ROSUnit_Arm(nh);
    ROSUnit* ros_updt_ctr = new ROSUnit_UpdateController(nh);
    ROSUnit* ros_updt_pose_ref = new ROSUnit_UpdatePoseReference(nh);
    ROSUnit* ros_pos_sub = new ROSUnit_PositionSubscriber(nh);
    ROSUnit* ros_rst_ctr = new ROSUnit_ResetController(nh);


    arm_motors->add_callback_msg_receiver((msg_receiver*) ros_arm_srv);
    disarm_motors->add_callback_msg_receiver((msg_receiver*) ros_arm_srv);
    update_controller->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    takeoff->add_callback_msg_receiver((msg_receiver*) ros_updt_pose_ref);
    land->add_callback_msg_receiver((msg_receiver*) ros_updt_pose_ref);
    reset_z->add_callback_msg_receiver((msg_receiver*) ros_rst_ctr);

    // ((UpdatePIDcontroller*)update_controller)->PIDdata.kp = 0.5;
    // ((UpdatePIDcontroller*)update_controller)->PIDdata.ki = 1;
    // ((UpdatePIDcontroller*)update_controller)->PIDdata.kd = 2;
    // ((UpdatePIDcontroller*)update_controller)->PIDdata.kdd = 0;
    // ((UpdatePIDcontroller*)update_controller)->PIDdata.anti_windup = 0;
    // ((UpdatePIDcontroller*)update_controller)->PIDdata.en_pv_derivation = 0;
    // ((UpdatePIDcontroller*)update_controller)->PIDdata.id = block_id::PID_ROLL;

    // UPDATE BEFORE FLIGHT
    ((UpdatePoseReference*)takeoff)->pose_reference.setPoseMessage(0.0, 0.0, 1.5, 1.54);
    ((UpdatePoseReference*)land)->pose_reference.setPoseMessage(0.0, 0.0, 0.4, 1.54);

    //**********************************************

    //First Pipeline
    Wait wait_1s;
    wait_1s.wait_time_ms=1000;
    Wait wait_10s;
    wait_10s.wait_time_ms=10000;

    FlightPipeline default_pipeline;
    default_pipeline.addElement((FlightElement*)reset_z);
    default_pipeline.addElement((FlightElement*)takeoff);
    default_pipeline.addElement((FlightElement*)&wait_1s);
    default_pipeline.addElement((FlightElement*)arm_motors);
    

    SimplePlaneCondition z_cross_takeoff;
    z_cross_takeoff.selected_dim=Dimension3D::Z;
    z_cross_takeoff.condition_value = 1.3;
    z_cross_takeoff.condition_met_for_larger=true;
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) &z_cross_takeoff);

    WaitForCondition z_cross_takeoff_check;
    z_cross_takeoff_check.Wait_condition=(Condition*)&z_cross_takeoff;

    SimplePlaneCondition z_cross_land;
    z_cross_land.selected_dim=Dimension3D::Z;
    z_cross_land.condition_value=0.4;
    z_cross_land.condition_met_for_larger=false;
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) &z_cross_land);

    WaitForCondition z_cross_land_check;
    z_cross_land_check.Wait_condition=(Condition*)&z_cross_land;


    FlightPipeline safety_pipeline;
    safety_pipeline.addElement((FlightElement*)&z_cross_takeoff_check);
    safety_pipeline.addElement((FlightElement*)&wait_10s);
    safety_pipeline.addElement((FlightElement*)land);
    safety_pipeline.addElement((FlightElement*)&z_cross_land_check);
    safety_pipeline.addElement((FlightElement*)&wait_1s);
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
