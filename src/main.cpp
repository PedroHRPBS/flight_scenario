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
#include "ROSUnit_InfoSubscriber.hpp"
#include "ChangeInternalState.hpp"
#include "InternalSystemStateCondition.hpp"
#include "StateMonitor.hpp"
#include "ROSUnit_Factory.hpp"
#include "ROSUnit_RestNormSettingsClnt.hpp"
#include "SetRestNormSettings.hpp"
#include "SetHeightOffset.hpp"
#include "SetRelativeWaypoint.hpp"

#undef TESTING
#undef MISSION
#define MRFT

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
    ROSUnit* ros_info_sub = new ROSUnit_InfoSubscriber(nh);
    ROSUnit* ros_restnorm_settings = new ROSUnit_RestNormSettingsClnt(nh);

    ROSUnit_Factory ROSUnit_Factory_main{nh};
	ROSUnit* ros_set_path_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Poses, "uav_control/set_path");
	ROSUnit* ros_set_hover_point_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Point, "uav_control/set_hover_point");
    //ROSUnit* ros_set_geofence_planes_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server_Publisher, ROSUnit_msg_type::ROSUnit_Points, "/uav_control/set_geofence_planes");
    ROSUnit* ros_set_mission_state_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Int, "uav_control/set_mission_state");
    ROSUnit* ros_uav_attitude_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, ROSUnit_msg_type::ROSUnit_Float, "uav_control/uav_attitude");
    ROSUnit* ros_updt_uav_control_state_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client, ROSUnit_msg_type::ROSUnit_Int, "ex_bldg_fire_mm/update_uav_control_state");
    ROSUnit* ros_set_height_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,ROSUnit_msg_type::ROSUnit_Float,"set_height_offset"); 

    //*****************Flight Elements*************

    FlightElement* update_controller_pid_x = new UpdateController();
    FlightElement* update_controller_pid_y = new UpdateController();
    FlightElement* update_controller_pid_z = new UpdateController();
    FlightElement* update_controller_pid_roll = new UpdateController();
    FlightElement* update_controller_pid_pitch = new UpdateController();
    FlightElement* update_controller_pid_yaw = new UpdateController();
    FlightElement* update_controller_pid_yaw_rate = new UpdateController();
    FlightElement* update_controller_pid_zero = new UpdateController();

    FlightElement* update_controller_mrft_x = new UpdateController();
    FlightElement* update_controller_mrft_y = new UpdateController();
    FlightElement* update_controller_mrft_z = new UpdateController();
    FlightElement* update_controller_mrft_roll = new UpdateController();
    FlightElement* update_controller_mrft_pitch = new UpdateController();
    FlightElement* update_controller_mrft_yaw = new UpdateController();
    FlightElement* update_controller_mrft_yaw_rate = new UpdateController();

    FlightElement* set_initial_pose = new SetInitialPose();

    FlightElement* switch_block_pid_mrft = new SwitchBlock();
    FlightElement* switch_block_mrft_pid = new SwitchBlock();
    
    FlightElement* reset_z = new ResetController();
    FlightElement* reset_y = new ResetController();
    
    FlightElement* arm_motors = new Arm();
    FlightElement* disarm_motors = new Disarm();

    FlightElement* ref_x = new SetReference_X();
    FlightElement* ref_y = new SetReference_Y();
    FlightElement* ref_z_on_takeoff = new SetReference_Z();
    FlightElement* ref_z_on_land = new SetReference_Z();
    FlightElement* ref_yaw = new SetReference_Yaw();

    FlightElement* flight_command = new FlightCommand();

    FlightElement* state_monitor = new StateMonitor();

    FlightElement* cs_to_hovering = new ChangeInternalState(uav_control_states::HOVERING);
    FlightElement* cs_to_landed = new ChangeInternalState(uav_control_states::LANDED);
    FlightElement* cs_to_taking_off = new ChangeInternalState(uav_control_states::TAKING_OFF);
    FlightElement* cs_to_landing = new ChangeInternalState(uav_control_states::LANDING);

    InternalSystemStateCondition* uav_control_taking_off = new InternalSystemStateCondition(uav_control_states::TAKING_OFF);
    WaitForCondition* taking_off_check = new WaitForCondition((Condition*)uav_control_taking_off);

    InternalSystemStateCondition* uav_control_landing = new InternalSystemStateCondition(uav_control_states::LANDING);
    WaitForCondition* landing_check = new WaitForCondition((Condition*)uav_control_landing);

    FlightElement* set_settings = new SetRestNormSettings(true, false, 0.5);
    FlightElement* set_height_offset = new SetHeightOffset();
    FlightElement* takeoff_relative_waypoint = new SetRelativeWaypoint(0., 0., 0.5, 0.);
    FlightElement* relative_waypoint_square_1 = new SetRelativeWaypoint(0.5, 0.5, 0.5, 0.);
    //FlightElement* takeoff_relative_waypoint = new SetRelativeWaypoint(0., 0., 0., 0.);
    //FlightElement* relative_waypoint_square_1 = new SetRelativeWaypoint(0.5, 0.5, 0., 0.);
    FlightElement* relative_waypoint_square_2 = new SetRelativeWaypoint(-2., -0.5, 0., 0.);
    FlightElement* relative_waypoint_square_3 = new SetRelativeWaypoint(-2., -0.5, 0., 0.);
    FlightElement* relative_waypoint_square_4 = new SetRelativeWaypoint(4., 1., 0., 0.);
    FlightElement* relative_waypoint_square_5 = new SetRelativeWaypoint(1., 0., 0., 0.);
    FlightElement* relative_waypoint_square_6 = new SetRelativeWaypoint(-0.5, -0.5, 0., 0.);
    FlightElement* land_relative_waypoint = new SetRelativeWaypoint(0., 0., -10., 0.);

    //******************Connections***************

    update_controller_pid_x->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_y->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_z->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_roll->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_pitch->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_yaw->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_yaw_rate->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_pid_zero->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);

    update_controller_mrft_x->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_mrft_y->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_mrft_z->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_mrft_roll->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_mrft_pitch->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_mrft_yaw->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);
    update_controller_mrft_yaw_rate->add_callback_msg_receiver((msg_receiver*) ros_updt_ctr);

    ros_ori_sub->add_callback_msg_receiver((msg_receiver*) set_initial_pose);
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) set_initial_pose);
    ros_ori_sub->add_callback_msg_receiver((msg_receiver*) takeoff_relative_waypoint);
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) takeoff_relative_waypoint);
    ros_ori_sub->add_callback_msg_receiver((msg_receiver*) relative_waypoint_square_1);
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) relative_waypoint_square_1);
    ros_ori_sub->add_callback_msg_receiver((msg_receiver*) relative_waypoint_square_2);
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) relative_waypoint_square_2);
    ros_ori_sub->add_callback_msg_receiver((msg_receiver*) relative_waypoint_square_3);
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) relative_waypoint_square_3);
    ros_ori_sub->add_callback_msg_receiver((msg_receiver*) relative_waypoint_square_4);
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) relative_waypoint_square_4);
    ros_ori_sub->add_callback_msg_receiver((msg_receiver*) relative_waypoint_square_5);
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) relative_waypoint_square_5);
    ros_ori_sub->add_callback_msg_receiver((msg_receiver*) relative_waypoint_square_6);
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) relative_waypoint_square_6);
    ros_ori_sub->add_callback_msg_receiver((msg_receiver*) land_relative_waypoint);
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) land_relative_waypoint);

    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) set_height_offset);

    set_initial_pose->add_callback_msg_receiver((msg_receiver*) ros_updt_x_ref);
    set_initial_pose->add_callback_msg_receiver((msg_receiver*) ros_updt_y_ref);
    set_initial_pose->add_callback_msg_receiver((msg_receiver*) ros_updt_z_ref);
    set_initial_pose->add_callback_msg_receiver((msg_receiver*) ros_updt_yaw_ref);

    switch_block_pid_mrft->add_callback_msg_receiver((msg_receiver*) ros_switch_block);
    switch_block_mrft_pid->add_callback_msg_receiver((msg_receiver*) ros_switch_block);

    reset_z->add_callback_msg_receiver((msg_receiver*) ros_rst_ctr);
    reset_y->add_callback_msg_receiver((msg_receiver*) ros_rst_ctr);

    arm_motors->add_callback_msg_receiver((msg_receiver*) ros_arm_srv);
    disarm_motors->add_callback_msg_receiver((msg_receiver*) ros_arm_srv);

    ref_x->add_callback_msg_receiver((msg_receiver*)ros_updt_x_ref);
    ref_y->add_callback_msg_receiver((msg_receiver*)ros_updt_y_ref);
    ref_z_on_takeoff->add_callback_msg_receiver((msg_receiver*)ros_updt_z_ref);
    ref_z_on_land->add_callback_msg_receiver((msg_receiver*)ros_updt_z_ref);
    ref_yaw->add_callback_msg_receiver((msg_receiver*)ros_updt_yaw_ref);

    ros_flight_command->add_callback_msg_receiver((msg_receiver*) flight_command);

    ros_set_mission_state_srv->add_callback_msg_receiver((msg_receiver*) cs_to_taking_off);
    ros_set_mission_state_srv->add_callback_msg_receiver((msg_receiver*) cs_to_landing);

    ros_updt_x_ref->add_callback_msg_receiver((msg_receiver*) state_monitor);
    ros_updt_y_ref->add_callback_msg_receiver((msg_receiver*) state_monitor);
    ros_updt_z_ref->add_callback_msg_receiver((msg_receiver*) state_monitor);
    ros_updt_yaw_ref->add_callback_msg_receiver((msg_receiver*) state_monitor);
    ros_info_sub->add_callback_msg_receiver((msg_receiver*) state_monitor);
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) state_monitor);

    state_monitor->add_callback_msg_receiver((msg_receiver*)ros_updt_uav_control_state_clnt);

    set_settings->add_callback_msg_receiver((msg_receiver*)ros_restnorm_settings);

    set_height_offset->add_callback_msg_receiver((msg_receiver*) ros_set_height_offset);
    takeoff_relative_waypoint->add_callback_msg_receiver((msg_receiver*) ros_set_path_clnt);
    relative_waypoint_square_1->add_callback_msg_receiver((msg_receiver*) ros_set_path_clnt);
    relative_waypoint_square_2->add_callback_msg_receiver((msg_receiver*) ros_set_path_clnt);
    relative_waypoint_square_3->add_callback_msg_receiver((msg_receiver*) ros_set_path_clnt);
    relative_waypoint_square_4->add_callback_msg_receiver((msg_receiver*) ros_set_path_clnt);
    relative_waypoint_square_5->add_callback_msg_receiver((msg_receiver*) ros_set_path_clnt);
    relative_waypoint_square_6->add_callback_msg_receiver((msg_receiver*) ros_set_path_clnt);
    land_relative_waypoint->add_callback_msg_receiver((msg_receiver*) ros_set_path_clnt);

    //*************Setting Flight Elements*************

    ((UpdateController*)update_controller_pid_zero)->pid_data.kp = 0.0;
    ((UpdateController*)update_controller_pid_zero)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_zero)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_zero)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_zero)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_zero)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_zero)->pid_data.id = block_id::PID_X;

    ((UpdateController*)update_controller_pid_x)->pid_data.kp = 0.51639;
    ((UpdateController*)update_controller_pid_x)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.kd = 0.21192;
    ((UpdateController*)update_controller_pid_x)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_x)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_x)->pid_data.id = block_id::PID_X;

    ((UpdateController*)update_controller_pid_y)->pid_data.kp = 0.51639;
    ((UpdateController*)update_controller_pid_y)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.kd = 0.21192;
    ((UpdateController*)update_controller_pid_y)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_y)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_y)->pid_data.id = block_id::PID_Y;

    ((UpdateController*)update_controller_pid_z)->pid_data.kp = 0.7450; 
    ((UpdateController*)update_controller_pid_z)->pid_data.ki = 0.0980; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kd = 0.3956; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_z)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_z)->pid_data.id = block_id::PID_Z;

    ((UpdateController*)update_controller_pid_roll)->pid_data.kp = 0.225; 
    ((UpdateController*)update_controller_pid_roll)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kd = 0.04;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_roll)->pid_data.id = block_id::PID_ROLL;

    ((UpdateController*)update_controller_pid_pitch)->pid_data.kp = 0.225; 
    ((UpdateController*)update_controller_pid_pitch)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kd = 0.04; 
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.id = block_id::PID_PITCH;

    ((UpdateController*)update_controller_pid_yaw)->pid_data.kp = 0.8*2;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.id = block_id::PID_YAW;

    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kp = 0.08*2;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.id = block_id::PID_YAW_RATE;

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
    ((UpdateController*)update_controller_mrft_roll)->mrft_data.relay_amp = 0.04;
    ((UpdateController*)update_controller_mrft_roll)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_roll)->mrft_data.id = block_id::MRFT_ROLL;

    ((UpdateController*)update_controller_mrft_pitch)->mrft_data.beta = -0.73;
    ((UpdateController*)update_controller_mrft_pitch)->mrft_data.relay_amp = 0.04;
    ((UpdateController*)update_controller_mrft_pitch)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_pitch)->mrft_data.id = block_id::MRFT_PITCH;

    ((UpdateController*)update_controller_mrft_yaw)->mrft_data.beta = -0.73;
    ((UpdateController*)update_controller_mrft_yaw)->mrft_data.relay_amp = 0.1;
    ((UpdateController*)update_controller_mrft_yaw)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_yaw)->mrft_data.id = block_id::MRFT_YAW;

    ((UpdateController*)update_controller_mrft_yaw_rate)->mrft_data.beta = -0.73;
    ((UpdateController*)update_controller_mrft_yaw_rate)->mrft_data.relay_amp = 0.1;
    ((UpdateController*)update_controller_mrft_yaw_rate)->mrft_data.bias = 0.0;
    ((UpdateController*)update_controller_mrft_yaw_rate)->mrft_data.id = block_id::MRFT_YAW_RATE;

    ((SwitchBlock*)switch_block_pid_mrft)->switch_msg.setSwitchBlockMsg_FS(block_id::PID_ROLL, block_id::MRFT_ROLL);
    ((SwitchBlock*)switch_block_mrft_pid)->switch_msg.setSwitchBlockMsg_FS(block_id::MRFT_ROLL, block_id::PID_ROLL);

    ((ResetController*)reset_z)->target_block = block_id::PID_Z;
    ((ResetController*)reset_y)->target_block = block_id::PID_Y;

    ((SetReference_Z*)ref_z_on_takeoff)->setpoint_z = 1.0;
    ((SetReference_Z*)ref_z_on_land)->setpoint_z = 0.0;

    Wait wait_1s;
    wait_1s.wait_time_ms=1000;

    Wait wait_100ms;
    wait_100ms.wait_time_ms=100;

    SimplePlaneCondition z_cross_takeoff_waypoint;
    z_cross_takeoff_waypoint.selected_dim=Dimension3D::Z;
    z_cross_takeoff_waypoint.condition_value = 0.9;
    z_cross_takeoff_waypoint.condition_met_for_larger=true;
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) &z_cross_takeoff_waypoint);

    WaitForCondition* z_cross_takeoff_waypoint_check = new WaitForCondition((Condition*)&z_cross_takeoff_waypoint);

    SimplePlaneCondition z_cross_land_waypoint;
    z_cross_land_waypoint.selected_dim=Dimension3D::Z;
    z_cross_land_waypoint.condition_value=0.1;
    z_cross_land_waypoint.condition_met_for_larger=false;
    ros_pos_sub->add_callback_msg_receiver((msg_receiver*) &z_cross_land_waypoint);

    WaitForCondition* z_cross_land_waypoint_check = new WaitForCondition((Condition*)&z_cross_land_waypoint);

    //**********************************************

    #ifdef MRFT
    FlightPipeline mrft_pipeline;

    mrft_pipeline.addElement((FlightElement*)&wait_1s);
    mrft_pipeline.addElement((FlightElement*)update_controller_pid_x);
    mrft_pipeline.addElement((FlightElement*)update_controller_pid_y);
    mrft_pipeline.addElement((FlightElement*)update_controller_pid_z);
    mrft_pipeline.addElement((FlightElement*)update_controller_pid_roll);
    mrft_pipeline.addElement((FlightElement*)update_controller_pid_pitch);
    mrft_pipeline.addElement((FlightElement*)update_controller_pid_yaw);
    mrft_pipeline.addElement((FlightElement*)update_controller_pid_yaw_rate);
    mrft_pipeline.addElement((FlightElement*)set_height_offset);
    mrft_pipeline.addElement((FlightElement*)&wait_1s);
    mrft_pipeline.addElement((FlightElement*)set_initial_pose);
    mrft_pipeline.addElement((FlightElement*)flight_command);
    mrft_pipeline.addElement((FlightElement*)reset_z);
    mrft_pipeline.addElement((FlightElement*)&wait_100ms);
    mrft_pipeline.addElement((FlightElement*)arm_motors);
    mrft_pipeline.addElement((FlightElement*)flight_command);
    mrft_pipeline.addElement((FlightElement*)set_settings);
    mrft_pipeline.addElement((FlightElement*)reset_z);
    mrft_pipeline.addElement((FlightElement*)takeoff_relative_waypoint);
    mrft_pipeline.addElement((FlightElement*)flight_command);
    mrft_pipeline.addElement((FlightElement*)update_controller_pid_zero);
    mrft_pipeline.addElement((FlightElement*)switch_block_pid_mrft);
    mrft_pipeline.addElement((FlightElement*)flight_command);
    mrft_pipeline.addElement((FlightElement*)update_controller_pid_x);
    mrft_pipeline.addElement((FlightElement*)switch_block_mrft_pid);
    mrft_pipeline.addElement((FlightElement*)flight_command);
    #endif
    
    #ifdef TESTING
    FlightPipeline testing_pipeline;

    testing_pipeline.addElement((FlightElement*)&wait_1s);
    testing_pipeline.addElement((FlightElement*)update_controller_pid_x);
    testing_pipeline.addElement((FlightElement*)update_controller_pid_y);
    testing_pipeline.addElement((FlightElement*)update_controller_pid_z);
    testing_pipeline.addElement((FlightElement*)update_controller_pid_roll);
    testing_pipeline.addElement((FlightElement*)update_controller_pid_pitch);
    testing_pipeline.addElement((FlightElement*)update_controller_pid_yaw);
    testing_pipeline.addElement((FlightElement*)update_controller_pid_yaw_rate);
    testing_pipeline.addElement((FlightElement*)set_height_offset);
    testing_pipeline.addElement((FlightElement*)&wait_1s);
    testing_pipeline.addElement((FlightElement*)set_initial_pose);
    testing_pipeline.addElement((FlightElement*)flight_command);
    testing_pipeline.addElement((FlightElement*)reset_z);
    testing_pipeline.addElement((FlightElement*)&wait_100ms);
    testing_pipeline.addElement((FlightElement*)arm_motors);
    testing_pipeline.addElement((FlightElement*)flight_command);
    testing_pipeline.addElement((FlightElement*)set_settings);
    testing_pipeline.addElement((FlightElement*)reset_z);
    testing_pipeline.addElement((FlightElement*)takeoff_relative_waypoint);
    testing_pipeline.addElement((FlightElement*)flight_command);
    testing_pipeline.addElement((FlightElement*)relative_waypoint_square_1);
    testing_pipeline.addElement((FlightElement*)flight_command);
    testing_pipeline.addElement((FlightElement*)relative_waypoint_square_2);
    testing_pipeline.addElement((FlightElement*)flight_command);
    testing_pipeline.addElement((FlightElement*)relative_waypoint_square_3);
    testing_pipeline.addElement((FlightElement*)flight_command);
    testing_pipeline.addElement((FlightElement*)relative_waypoint_square_4);
    testing_pipeline.addElement((FlightElement*)flight_command);
    testing_pipeline.addElement((FlightElement*)relative_waypoint_square_5);
    testing_pipeline.addElement((FlightElement*)flight_command);
    testing_pipeline.addElement((FlightElement*)relative_waypoint_square_6);
    testing_pipeline.addElement((FlightElement*)flight_command);
    testing_pipeline.addElement((FlightElement*)land_relative_waypoint);
    #endif

    #ifdef MISSION
    FlightPipeline initialization_pipeline, state_monitor_pipeline, take_off_pipeline, landing_pipeline;

    //The Wait is needed because otherwise the set_initial_pose will capture only zeros
    initialization_pipeline.addElement((FlightElement*)&wait_1s);
    initialization_pipeline.addElement((FlightElement*)set_initial_pose);
    initialization_pipeline.addElement((FlightElement*)update_controller_pid_x);
    initialization_pipeline.addElement((FlightElement*)update_controller_pid_y);
    initialization_pipeline.addElement((FlightElement*)update_controller_pid_z);
    initialization_pipeline.addElement((FlightElement*)update_controller_pid_roll);
    initialization_pipeline.addElement((FlightElement*)update_controller_pid_pitch);
    initialization_pipeline.addElement((FlightElement*)update_controller_pid_yaw);
    initialization_pipeline.addElement((FlightElement*)update_controller_pid_yaw_rate);
    initialization_pipeline.addElement((FlightElement*)update_controller_mrft_x);
    initialization_pipeline.addElement((FlightElement*)update_controller_mrft_y);
    initialization_pipeline.addElement((FlightElement*)update_controller_mrft_z);
    initialization_pipeline.addElement((FlightElement*)update_controller_mrft_roll);
    initialization_pipeline.addElement((FlightElement*)update_controller_mrft_pitch);
    initialization_pipeline.addElement((FlightElement*)update_controller_mrft_yaw);
    initialization_pipeline.addElement((FlightElement*)update_controller_mrft_yaw_rate);
    //-----------
    take_off_pipeline.addElement((FlightElement*)taking_off_check);
    take_off_pipeline.addElement((FlightElement*)reset_z);
    take_off_pipeline.addElement((FlightElement*)arm_motors);
    take_off_pipeline.addElement((FlightElement*)set_settings);
    take_off_pipeline.addElement((FlightElement*)cs_to_hovering);
    //-----------
    landing_pipeline.addElement((FlightElement*)landing_check);
    landing_pipeline.addElement((FlightElement*)ref_z_on_land);
    landing_pipeline.addElement((FlightElement*)z_cross_land_waypoint_check);
    landing_pipeline.addElement((FlightElement*)&wait_1s);
    landing_pipeline.addElement((FlightElement*)disarm_motors);
    landing_pipeline.addElement((FlightElement*)cs_to_landed);
    //-----------
    state_monitor_pipeline.addElement((FlightElement*)state_monitor);
    //-----------  
    // safety_pipeline.addElement((FlightElement*)&z_cross_takeoff_waypoint_check);
    // safety_pipeline.addElement((FlightElement*)ref_z_on_takeoff);
    // safety_pipeline.addElement((FlightElement*)&z_cross_land_waypoint_check);
    // safety_pipeline.addElement((FlightElement*)&wait_1s);
    // safety_pipeline.addElement((FlightElement*)disarm_motors);
    #endif
    Logger::getAssignedLogger()->log("FlightScenario main_scenario",LoggerLevel::Info);
    FlightScenario main_scenario;
    #ifdef MRFT
    main_scenario.AddFlightPipeline(&mrft_pipeline);
    #endif
    #ifdef TESTING
    main_scenario.AddFlightPipeline(&testing_pipeline);
    #endif
    #ifdef MISSION
    main_scenario.AddFlightPipeline(&initialization_pipeline);
    main_scenario.AddFlightPipeline(&take_off_pipeline);
    main_scenario.AddFlightPipeline(&landing_pipeline);
    main_scenario.AddFlightPipeline(&state_monitor_pipeline);
    #endif
    main_scenario.StartScenario();
    Logger::getAssignedLogger()->log("Main Done",LoggerLevel::Info);
    
    while(ros::ok){
        ros::spinOnce();
    }
}
