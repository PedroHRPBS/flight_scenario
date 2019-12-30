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
#include "UpdateController.hpp"
#include "SetInitialPose.hpp"
#include "ResetController.hpp"
#include "SwitchBlock.hpp"
#include "ROSUnit_Arm.hpp"
#include "ROSUnit_UpdateController.hpp"
#include "ROSUnit_UpdatePoseReference.hpp"
#include "ROSUnit_PositionSubscriber.hpp"
#include "ROSUnit_ResetController.hpp"
#include "ROSUnit_SwitchBlock.hpp"
#include "ROSUnit_OrientationSubscriber.hpp"
#include "ROSUnit_UpdateReferenceX_FS.hpp"
#include "ROSUnit_UpdateReferenceY_FS.hpp"
#include "ROSUnit_UpdateReferenceZ_FS.hpp"
#include "ROSUnit_UpdateReferenceYaw_FS.hpp"
#include "SetReference_X.hpp"
#include "SetReference_Y.hpp"
#include "SetReference_Z.hpp"
#include "SetReference_Yaw.hpp"
#include "ROSUnit_FlightCommand.hpp"
#include "FlightCommand.hpp"

int main(int argc, char** argv) {
    Logger::assignLogger(new StdLogger());

    //****************ROS Units********************
    ros::init(argc, argv, "flight_scenario");
    ros::NodeHandle nh;

    ROSUnit* ros_arm_srv = new ROSUnit_Arm(nh);
    ROSUnit* ros_updt_ctr = new ROSUnit_UpdateController(nh);
    ROSUnit* ros_pos_sub = new ROSUnit_PositionSubscriber(nh);
    ROSUnit* ros_ori_sub = new ROSUnit_OrientationSubscriber(nh);
    ROSUnit* ros_rst_ctr = new ROSUnit_ResetController(nh);
    ROSUnit* ros_switch_block = new ROSUnit_SwitchBlock(nh);
    ROSUnit* ros_updt_x_ref = new ROSUnit_UpdateReferenceX_FS(nh);
    ROSUnit* ros_updt_y_ref = new ROSUnit_UpdateReferenceY_FS(nh);
    ROSUnit* ros_updt_z_ref = new ROSUnit_UpdateReferenceZ_FS(nh);
    ROSUnit* ros_updt_yaw_ref = new ROSUnit_UpdateReferenceYaw_FS(nh);
    ROSUnit* ros_flight_command = new ROSUnit_FlightCommand(nh);

    //*****************Flight Elements*************

    FlightElement* update_controller_pid_x = new UpdateController();
    FlightElement* update_controller_pid_y = new UpdateController();
    FlightElement* update_controller_pid_z = new UpdateController();
    FlightElement* update_controller_pid_roll = new UpdateController();
    FlightElement* update_controller_pid_pitch = new UpdateController();
    FlightElement* update_controller_pid_yaw = new UpdateController();

    FlightElement* update_controller_mrft_x = new UpdateController();
    FlightElement* update_controller_mrft_y = new UpdateController();
    FlightElement* update_controller_mrft_z = new UpdateController();
    FlightElement* update_controller_mrft_roll = new UpdateController();
    FlightElement* update_controller_mrft_pitch = new UpdateController();
    FlightElement* update_controller_mrft_yaw = new UpdateController();

    FlightElement* set_initial_pose = new SetInitialPose();

    FlightElement* switch_block = new SwitchBlock();
    
    FlightElement* reset_z = new ResetController();

    FlightElement* arm_motors = new Arm();
    FlightElement* disarm_motors = new Disarm();

    FlightElement* ref_x = new SetReference_X();
    FlightElement* ref_y = new SetReference_Y();
    FlightElement* ref_z = new SetReference_Z();
    FlightElement* ref_yaw = new SetReference_Yaw();

    FlightElement* flight_command = new FlightCommand();

    //******************Connectionerences***************

    update_controller_pid_x->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_y->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_z->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_roll->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_pitch->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_yaw->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);

    update_controller_mrft_x->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_mrft_y->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_mrft_z->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_mrft_roll->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_mrft_pitch->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_mrft_yaw->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);

    ros_ori_sub->add_callback_msg_receiver((msg_receiver*) set_initial_pose);
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) set_initial_pose);

    set_initial_pose->add_callback_msg_receiver((msg_receiver*) ros_updt_x_ref);
    set_initial_pose->add_callback_msg_receiver((msg_receiver*) ros_updt_y_ref);
    set_initial_pose->add_callback_msg_receiver((msg_receiver*) ros_updt_z_ref);
    set_initial_pose->add_callback_msg_receiver((msg_receiver*) ros_updt_yaw_ref);

    switch_block->add_callback_msg_receiver((msg_receiver*) ros_switch_block);
    //TODO Should I implement a reset controller for MRFT??
    reset_z->add_callback_msg_receiver((msg_receiver*) ros_rst_ctr);

    arm_motors->add_callback_msg_receiver((msg_receiver*) ros_arm_srv);
    disarm_motors->add_callback_msg_receiver((msg_receiver*) ros_arm_srv);

    ref_x->add_callback_msg_receiver((msg_receiver*)ros_updt_x_ref);
    ref_y->add_callback_msg_receiver((msg_receiver*)ros_updt_y_ref);
    ref_z->add_callback_msg_receiver((msg_receiver*)ros_updt_z_ref);
    ref_yaw->add_callback_msg_receiver((msg_receiver*)ros_updt_yaw_ref);

    ros_flight_command->add_callback_msg_receiver((msg_receiver*) flight_command);

    //*************Setting Flight Elements*************

    ((UpdateController*)update_controller_pid_x)->pid_data.kp = 0.8;
    ((UpdateController*)update_controller_pid_x)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.kd = 0.6;
    ((UpdateController*)update_controller_pid_x)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_x)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_x)->pid_data.id = block_id::PID_X;

    ((UpdateController*)update_controller_pid_y)->pid_data.kp = 0.8;
    ((UpdateController*)update_controller_pid_y)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.kd = 0.6;
    ((UpdateController*)update_controller_pid_y)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_y)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_y)->pid_data.id = block_id::PID_Y;

    ((UpdateController*)update_controller_pid_z)->pid_data.kp = 0.4;
    ((UpdateController*)update_controller_pid_z)->pid_data.ki = 0.04;
    ((UpdateController*)update_controller_pid_z)->pid_data.kd = 0.10;
    ((UpdateController*)update_controller_pid_z)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_z)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_z)->pid_data.id = block_id::PID_Z;

    ((UpdateController*)update_controller_pid_roll)->pid_data.kp = 0.3;
    ((UpdateController*)update_controller_pid_roll)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kd = 0.075;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_roll)->pid_data.id = block_id::PID_ROLL;

    ((UpdateController*)update_controller_pid_pitch)->pid_data.kp = 0.3;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kd = 0.075;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.id = block_id::PID_PITCH;

    ((UpdateController*)update_controller_pid_yaw)->pid_data.kp = 0.8;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kd = 0.08;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.id = block_id::PID_YAW;

    ((UpdateController*)update_controller_mrft_x)->mrft_data.beta = -0.73;
    ((UpdateController*)update_controller_mrft_x)->mrft_data.relay_amp = 0.1;
    ((UpdateController*)update_controller_mrft_x)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_x)->mrft_data.id = block_id::MRFT_X;

    ((UpdateController*)update_controller_mrft_y)->mrft_data.beta = -0.73;
    ((UpdateController*)update_controller_mrft_y)->mrft_data.relay_amp = 0.1;
    ((UpdateController*)update_controller_mrft_y)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_y)->mrft_data.id = block_id::MRFT_Y;

    ((UpdateController*)update_controller_mrft_z)->mrft_data.beta = -0.73;
    ((UpdateController*)update_controller_mrft_z)->mrft_data.relay_amp = 0.1;
    ((UpdateController*)update_controller_mrft_z)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_z)->mrft_data.id = block_id::MRFT_Z;
    
    ((UpdateController*)update_controller_mrft_roll)->mrft_data.beta = -0.73;
    ((UpdateController*)update_controller_mrft_roll)->mrft_data.relay_amp = 0.05;
    ((UpdateController*)update_controller_mrft_roll)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_roll)->mrft_data.id = block_id::MRFT_ROLL;

    ((UpdateController*)update_controller_mrft_pitch)->mrft_data.beta = -0.73;
    ((UpdateController*)update_controller_mrft_pitch)->mrft_data.relay_amp = 0.05;
    ((UpdateController*)update_controller_mrft_pitch)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_pitch)->mrft_data.id = block_id::MRFT_PITCH;

    ((UpdateController*)update_controller_mrft_yaw)->mrft_data.beta = -0.73;
    ((UpdateController*)update_controller_mrft_yaw)->mrft_data.relay_amp = 0.1;
    ((UpdateController*)update_controller_mrft_yaw)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_yaw)->mrft_data.id = block_id::MRFT_YAW;

    ((SwitchBlock*)switch_block)->switch_msg.setSwitchBlockMsg_FS(block_id::PID_ROLL, block_id::MRFT_ROLL);

    ((ResetController*)reset_z)->target_block = block_id::PID_Z;

    //((SetReference_X*)ref_x)->setpoint_x = 1;
    //((SetReference_Y*)ref_y)->setpoint_y = 2;
    ((SetReference_Z*)ref_z)->setpoint_z = 1.5;
    //((SetReference_Yaw*)ref_yaw)->setpoint_yaw = 4;

    //First Pipeline
    Wait wait_1s;
    wait_1s.wait_time_ms=1000;
    Wait wait_5s;
    wait_5s.wait_time_ms=5000;
    Wait wait_2s;
    wait_2s.wait_time_ms=2000;

    SimplePlaneCondition z_cross_takeoff_waypoint;
    z_cross_takeoff_waypoint.selected_dim=Dimension3D::Z;
    z_cross_takeoff_waypoint.condition_value = 1.0;
    z_cross_takeoff_waypoint.condition_met_for_larger=true;
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) &z_cross_takeoff_waypoint);

    WaitForCondition z_cross_takeoff_waypoint_check;
    z_cross_takeoff_waypoint_check.Wait_condition=(Condition*)&z_cross_takeoff_waypoint;

    SimplePlaneCondition z_cross_land_waypoint;
    z_cross_land_waypoint.selected_dim=Dimension3D::Z;
    z_cross_land_waypoint.condition_value=0.4;
    z_cross_land_waypoint.condition_met_for_larger=false;
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) &z_cross_land_waypoint);

    WaitForCondition z_cross_land_waypoint_check;
    z_cross_land_waypoint_check.Wait_condition=(Condition*)&z_cross_land_waypoint;

    //**********************************************

    

    FlightPipeline default_pipeline;

    //The Wait is needed because otherwise the set_initial_pose will capture only zeros
    default_pipeline.addElement((FlightElement*)&wait_1s);
    default_pipeline.addElement((FlightElement*)set_initial_pose);
    
    default_pipeline.addElement((FlightElement*)update_controller_pid_x);
    default_pipeline.addElement((FlightElement*)update_controller_pid_y);
    default_pipeline.addElement((FlightElement*)update_controller_pid_z);
    default_pipeline.addElement((FlightElement*)update_controller_pid_roll);
    default_pipeline.addElement((FlightElement*)update_controller_pid_pitch);
    default_pipeline.addElement((FlightElement*)update_controller_pid_yaw);

    default_pipeline.addElement((FlightElement*)update_controller_mrft_x);
    default_pipeline.addElement((FlightElement*)update_controller_mrft_y);
    default_pipeline.addElement((FlightElement*)update_controller_mrft_z);
    default_pipeline.addElement((FlightElement*)update_controller_mrft_roll);
    default_pipeline.addElement((FlightElement*)update_controller_mrft_pitch);
    default_pipeline.addElement((FlightElement*)update_controller_mrft_yaw);

    default_pipeline.addElement((FlightElement*)flight_command);
    
    default_pipeline.addElement((FlightElement*)ref_z);
    default_pipeline.addElement((FlightElement*)reset_z);
    default_pipeline.addElement((FlightElement*)arm_motors);

    //default_pipeline.addElement((FlightElement*)&z_cross_takeoff_waypoint_check);

    default_pipeline.addElement((FlightElement*)flight_command);
    
    default_pipeline.addElement((FlightElement*)switch_block);

    default_pipeline.addElement((FlightElement*)flight_command);

    ((SwitchBlock*)switch_block)->switch_msg.setSwitchBlockMsg_FS(block_id::MRFT_ROLL, block_id::PID_ROLL);
    default_pipeline.addElement((FlightElement*)switch_block);

    default_pipeline.addElement((FlightElement*)flight_command);

    default_pipeline.addElement((FlightElement*)&z_cross_land_waypoint_check);
    default_pipeline.addElement((FlightElement*)&wait_1s);
    default_pipeline.addElement((FlightElement*)disarm_motors);

    FlightPipeline safety_pipeline;
    // safety_pipeline.addElement((FlightElement*)&z_cross_takeoff_waypoint_check);
    // safety_pipeline.addElement((FlightElement*)ref_z);
    // safety_pipeline.addElement((FlightElement*)&z_cross_land_waypoint_check);
    // safety_pipeline.addElement((FlightElement*)&wait_1s);
    // safety_pipeline.addElement((FlightElement*)disarm_motors);
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
