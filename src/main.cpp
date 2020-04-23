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
#include "ResetController.hpp"
#include "SwitchBlock.hpp"
#include "ROSUnit_Arm.hpp"
#include "ROSUnit_UpdateController.hpp"
#include "ROSUnit_PositionSubscriber.hpp"
#include "ROSUnit_ResetController.hpp"
#include "ROSUnit_SwitchBlock.hpp"
#include "ROSUnit_OrientationSubscriber.hpp"
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
#include "ROSUnit_ControlOutputSubscriber.hpp"

#undef TESTING
#undef MISSION
#undef MRFT
#define MRFT_TAKEOFF_DNN

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
    ROSUnit* ros_flight_command = new ROSUnit_FlightCommand(nh);
    ROSUnit* ros_info_sub = new ROSUnit_InfoSubscriber(nh);
    ROSUnit* ros_restnorm_settings = new ROSUnit_RestNormSettingsClnt(nh);
    

    ROSUnit_Factory ROSUnit_Factory_main{nh};
	ROSUnit* ros_set_path_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                    ROSUnit_msg_type::ROSUnit_Poses,
                                                                    "uav_control/set_path");
	ROSUnit* ros_set_hover_point_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "uav_control/set_hover_point");
    ROSUnit* ros_set_mission_state_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                            ROSUnit_msg_type::ROSUnit_Int,
                                                                            "uav_control/set_mission_state");
    ROSUnit* ros_uav_attitude_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "uav_control/uav_attitude");
    ROSUnit* ros_updt_uav_control_state_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                                ROSUnit_msg_type::ROSUnit_Int,
                                                                                "ex_bldg_fire_mm/update_uav_control_state");
    ROSUnit* ros_set_height_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "set_height_offset"); 
    ROSUnit* rosunit_x_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/x");
    ROSUnit* rosunit_y_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/y");
    ROSUnit* rosunit_z_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/z");
    ROSUnit* rosunit_roll_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/roll");
    ROSUnit* rosunit_pitch_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/pitch");
    ROSUnit* rosunit_yaw_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw");
    ROSUnit* rosunit_yaw_rate_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw_rate");

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

    FlightElement* update_controller_sm_x = new UpdateController();
    FlightElement* update_controller_sm_y = new UpdateController();


    FlightElement* switch_block_pid_mrft = new SwitchBlock();
    FlightElement* switch_block_mrft_pid = new SwitchBlock();
    
    FlightElement* reset_z = new ResetController();
    FlightElement* reset_x = new ResetController();
    
    FlightElement* arm_motors = new Arm();
    FlightElement* disarm_motors = new Disarm();

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

    FlightElement* set_settings = new SetRestNormSettings(true, false, 0.25);
    FlightElement* set_height_offset = new SetHeightOffset();
    FlightElement* initial_pose_waypoint = new SetRelativeWaypoint(0., 0., 0., 0.);
    FlightElement* takeoff_relative_waypoint = new SetRelativeWaypoint(0., 0., 2.0, 0.); //DNN: this is set to 2m height
    FlightElement* relative_waypoint_square_1 = new SetRelativeWaypoint(1.0, 0.0, 0.5, 0.);
    FlightElement* relative_waypoint_square_2 = new SetRelativeWaypoint(.0, 1.0, 0., 0.);
    FlightElement* relative_waypoint_square_3 = new SetRelativeWaypoint(-2., 0., 0., 0.);
    FlightElement* relative_waypoint_square_4 = new SetRelativeWaypoint(0.0, -2., 0., 0.);
    FlightElement* relative_waypoint_square_5 = new SetRelativeWaypoint(2., 0., 0., 0.);
    FlightElement* relative_waypoint_square_6 = new SetRelativeWaypoint(-1.0, -1.0, 0., 0.);
    FlightElement* land_relative_waypoint = new SetRelativeWaypoint(0., 0., -10., 0.);

    //******************Connections***************

    update_controller_pid_x->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::pid);
    update_controller_pid_y->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::pid);
    update_controller_pid_z->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::pid);
    update_controller_pid_roll->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::pid);
    update_controller_pid_pitch->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::pid);
    update_controller_pid_yaw->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::pid);
    update_controller_pid_yaw_rate->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::pid);
    update_controller_pid_zero->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::pid);

    update_controller_mrft_x->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::mrft);
    update_controller_mrft_y->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::mrft);
    update_controller_mrft_z->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::mrft);
    update_controller_mrft_roll->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::mrft);
    update_controller_mrft_pitch->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::mrft);
    update_controller_mrft_yaw->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::mrft);
    update_controller_mrft_yaw_rate->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::mrft);

    update_controller_sm_x->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::sm);
    update_controller_sm_y->setEmittingChannel(ROSUnit_UpdateController::receiving_channels::sm);

    update_controller_pid_x->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);
    update_controller_pid_y->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);
    update_controller_pid_z->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);
    update_controller_pid_roll->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);
    update_controller_pid_pitch->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);
    update_controller_pid_yaw->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);
    update_controller_pid_yaw_rate->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);
    update_controller_pid_zero->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);

    update_controller_mrft_x->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);
    update_controller_mrft_y->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);
    update_controller_mrft_z->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);
    update_controller_mrft_roll->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);
    update_controller_mrft_pitch->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);
    update_controller_mrft_yaw->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);
    update_controller_mrft_yaw_rate->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);

    update_controller_sm_x->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);
    update_controller_sm_y->addCallbackMsgReceiver((MsgReceiver*) ros_updt_ctr);


    ros_ori_sub->addCallbackMsgReceiver((MsgReceiver*) initial_pose_waypoint);
    ros_pos_sub->addCallbackMsgReceiver((MsgReceiver*) initial_pose_waypoint);
    ros_ori_sub->addCallbackMsgReceiver((MsgReceiver*) takeoff_relative_waypoint);
    ros_pos_sub->addCallbackMsgReceiver((MsgReceiver*) takeoff_relative_waypoint);
    ros_ori_sub->addCallbackMsgReceiver((MsgReceiver*) relative_waypoint_square_1);
    ros_pos_sub->addCallbackMsgReceiver((MsgReceiver*) relative_waypoint_square_1);
    ros_ori_sub->addCallbackMsgReceiver((MsgReceiver*) relative_waypoint_square_2);
    ros_pos_sub->addCallbackMsgReceiver((MsgReceiver*) relative_waypoint_square_2);
    ros_ori_sub->addCallbackMsgReceiver((MsgReceiver*) relative_waypoint_square_3);
    ros_pos_sub->addCallbackMsgReceiver((MsgReceiver*) relative_waypoint_square_3);
    ros_ori_sub->addCallbackMsgReceiver((MsgReceiver*) relative_waypoint_square_4);
    ros_pos_sub->addCallbackMsgReceiver((MsgReceiver*) relative_waypoint_square_4);
    ros_ori_sub->addCallbackMsgReceiver((MsgReceiver*) relative_waypoint_square_5);
    ros_pos_sub->addCallbackMsgReceiver((MsgReceiver*) relative_waypoint_square_5);
    ros_ori_sub->addCallbackMsgReceiver((MsgReceiver*) relative_waypoint_square_6);
    ros_pos_sub->addCallbackMsgReceiver((MsgReceiver*) relative_waypoint_square_6);
    ros_ori_sub->addCallbackMsgReceiver((MsgReceiver*) land_relative_waypoint);
    ros_pos_sub->addCallbackMsgReceiver((MsgReceiver*) land_relative_waypoint);

    ros_pos_sub->addCallbackMsgReceiver((MsgReceiver*) set_height_offset);

    switch_block_pid_mrft->addCallbackMsgReceiver((MsgReceiver*) ros_switch_block);
    switch_block_mrft_pid->addCallbackMsgReceiver((MsgReceiver*) ros_switch_block);

    reset_z->addCallbackMsgReceiver((MsgReceiver*) ros_rst_ctr);
    reset_x->addCallbackMsgReceiver((MsgReceiver*) ros_rst_ctr);

    arm_motors->addCallbackMsgReceiver((MsgReceiver*) ros_arm_srv);
    disarm_motors->addCallbackMsgReceiver((MsgReceiver*) ros_arm_srv);

    ros_flight_command->addCallbackMsgReceiver((MsgReceiver*) flight_command);

    ros_set_mission_state_srv->addCallbackMsgReceiver((MsgReceiver*) cs_to_taking_off);
    ros_set_mission_state_srv->addCallbackMsgReceiver((MsgReceiver*) cs_to_landing);

    ros_info_sub->addCallbackMsgReceiver((MsgReceiver*) state_monitor);
    ros_pos_sub->addCallbackMsgReceiver((MsgReceiver*) state_monitor);

    state_monitor->addCallbackMsgReceiver((MsgReceiver*)ros_updt_uav_control_state_clnt);

    set_settings->addCallbackMsgReceiver((MsgReceiver*)ros_restnorm_settings);

    set_height_offset->addCallbackMsgReceiver((MsgReceiver*) ros_set_height_offset);
    initial_pose_waypoint->addCallbackMsgReceiver((MsgReceiver*) ros_set_path_clnt);
    takeoff_relative_waypoint->addCallbackMsgReceiver((MsgReceiver*) ros_set_path_clnt);
    relative_waypoint_square_1->addCallbackMsgReceiver((MsgReceiver*) ros_set_path_clnt);
    relative_waypoint_square_2->addCallbackMsgReceiver((MsgReceiver*) ros_set_path_clnt);
    relative_waypoint_square_3->addCallbackMsgReceiver((MsgReceiver*) ros_set_path_clnt);
    relative_waypoint_square_4->addCallbackMsgReceiver((MsgReceiver*) ros_set_path_clnt);
    relative_waypoint_square_5->addCallbackMsgReceiver((MsgReceiver*) ros_set_path_clnt);
    relative_waypoint_square_6->addCallbackMsgReceiver((MsgReceiver*) ros_set_path_clnt);
    land_relative_waypoint->addCallbackMsgReceiver((MsgReceiver*) ros_set_path_clnt);

    //*************Setting Flight Elements*************

    ((UpdateController*)update_controller_pid_zero)->pid_data.kp = 0.0;
    ((UpdateController*)update_controller_pid_zero)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_zero)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_zero)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_zero)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_zero)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_zero)->pid_data.id = block_id::PID_X;

    ((UpdateController*)update_controller_pid_x)->pid_data.kp = 0.51639 * 0.8;
    ((UpdateController*)update_controller_pid_x)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.kd = 0.21192 * 0.8;
    ((UpdateController*)update_controller_pid_x)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_x)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_x)->pid_data.id = block_id::PID_X;

    ((UpdateController*)update_controller_pid_y)->pid_data.kp = 0.51639 * 0.8;
    ((UpdateController*)update_controller_pid_y)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.kd = 0.21192 * 0.8;
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

    ((UpdateController*)update_controller_pid_roll)->pid_data.kp = 0.225 * 0.8; 
    ((UpdateController*)update_controller_pid_roll)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kd = 0.04 * 0.8;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_roll)->pid_data.id = block_id::PID_ROLL;

    ((UpdateController*)update_controller_pid_pitch)->pid_data.kp = 0.225 * 0.8; 
    ((UpdateController*)update_controller_pid_pitch)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kd = 0.04 * 0.8; 
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.id = block_id::PID_PITCH;

    ((UpdateController*)update_controller_pid_yaw)->pid_data.kp = 1.6;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.id = block_id::PID_YAW;

    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kp = 0.16 * 0.5; //TODO check this new value, trying to reduce the jumps while flying.
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

    ((UpdateController*)update_controller_sm_x)->sm_data.alpha1 = 0.3;
    ((UpdateController*)update_controller_sm_x)->sm_data.alpha2 = 1.0;
    ((UpdateController*)update_controller_sm_x)->sm_data.h1 = 1.5;
    ((UpdateController*)update_controller_sm_x)->sm_data.h2 = 2.0;
    ((UpdateController*)update_controller_sm_x)->sm_data.id = block_id::SM_X;

    ((UpdateController*)update_controller_sm_y)->sm_data.alpha1 = 0.3;
    ((UpdateController*)update_controller_sm_y)->sm_data.alpha2 = 1.0;
    ((UpdateController*)update_controller_sm_y)->sm_data.h1 = 1.5;
    ((UpdateController*)update_controller_sm_y)->sm_data.h2 = 2.0;
    ((UpdateController*)update_controller_sm_y)->sm_data.id = block_id::SM_Y;

    ((SwitchBlock*)switch_block_pid_mrft)->switch_msg.setSwitchBlockMsg_FS(block_id::PID_ROLL, block_id::MRFT_ROLL);
    ((SwitchBlock*)switch_block_mrft_pid)->switch_msg.setSwitchBlockMsg_FS(block_id::MRFT_ROLL, block_id::PID_ROLL);

    ((ResetController*)reset_z)->target_block = block_id::PID_Z;
    ((ResetController*)reset_x)->target_block = block_id::PID_X;

    Wait wait_1s;
    wait_1s.wait_time_ms=1000;

    Wait wait_100ms;
    wait_100ms.wait_time_ms=100;

    SimplePlaneCondition z_cross_takeoff_waypoint;
    z_cross_takeoff_waypoint.selected_dim=Dimension3D::Z;
    z_cross_takeoff_waypoint.condition_value = 0.9;
    z_cross_takeoff_waypoint.condition_met_for_larger=true;
    ros_pos_sub->addCallbackMsgReceiver((MsgReceiver*) &z_cross_takeoff_waypoint);

    WaitForCondition* z_cross_takeoff_waypoint_check = new WaitForCondition((Condition*)&z_cross_takeoff_waypoint);

    SimplePlaneCondition z_cross_land_waypoint;
    z_cross_land_waypoint.selected_dim=Dimension3D::Z;
    z_cross_land_waypoint.condition_value=0.1;
    z_cross_land_waypoint.condition_met_for_larger=false;
    ros_pos_sub->addCallbackMsgReceiver((MsgReceiver*) &z_cross_land_waypoint);

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
    mrft_pipeline.addElement((FlightElement*)update_controller_mrft_x);
    mrft_pipeline.addElement((FlightElement*)update_controller_mrft_y);
    mrft_pipeline.addElement((FlightElement*)update_controller_mrft_z);
    mrft_pipeline.addElement((FlightElement*)update_controller_mrft_roll);
    mrft_pipeline.addElement((FlightElement*)update_controller_mrft_pitch);
    mrft_pipeline.addElement((FlightElement*)update_controller_mrft_yaw);
    mrft_pipeline.addElement((FlightElement*)update_controller_mrft_yaw_rate);
    mrft_pipeline.addElement((FlightElement*)set_height_offset);
    mrft_pipeline.addElement((FlightElement*)&wait_1s);
    //mrft_pipeline.addElement((FlightElement*)set_initial_pose);
    mrft_pipeline.addElement((FlightElement*)flight_command);
    mrft_pipeline.addElement((FlightElement*)reset_z);
    mrft_pipeline.addElement((FlightElement*)&wait_100ms);
    mrft_pipeline.addElement((FlightElement*)arm_motors);
    mrft_pipeline.addElement((FlightElement*)flight_command);
    mrft_pipeline.addElement((FlightElement*)set_settings);
    mrft_pipeline.addElement((FlightElement*)reset_z);
    mrft_pipeline.addElement((FlightElement*)takeoff_relative_waypoint);
    mrft_pipeline.addElement((FlightElement*)flight_command);
    mrft_pipeline.addElement((FlightElement*)switch_block_pid_mrft);
    mrft_pipeline.addElement((FlightElement*)update_controller_pid_zero);
    mrft_pipeline.addElement((FlightElement*)flight_command);
    mrft_pipeline.addElement((FlightElement*)switch_block_mrft_pid);
    //mrft_pipeline.addElement((FlightElement*)set_initial_pose);
    mrft_pipeline.addElement((FlightElement*)update_controller_pid_x);
    mrft_pipeline.addElement((FlightElement*)reset_x);

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
    testing_pipeline.addElement((FlightElement*)set_settings);
    testing_pipeline.addElement((FlightElement*)initial_pose_waypoint);
    testing_pipeline.addElement((FlightElement*)flight_command);
    testing_pipeline.addElement((FlightElement*)reset_z);
    testing_pipeline.addElement((FlightElement*)&wait_100ms);
    testing_pipeline.addElement((FlightElement*)arm_motors);
    testing_pipeline.addElement((FlightElement*)flight_command);
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
    initialization_pipeline.addElement((FlightElement*)set_height_offset);
    initialization_pipeline.addElement((FlightElement*)&wait_1s);
    initialization_pipeline.addElement((FlightElement*)set_initial_pose);
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

    #ifdef MRFT_TAKEOFF_DNN
    FlightPipeline mrft_takeoff_dnn_pipeline;

    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)&wait_1s);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)update_controller_sm_x);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)update_controller_sm_y);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)update_controller_pid_z); //Both need to be updated, because together they become one
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)update_controller_mrft_z);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)update_controller_mrft_roll);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)update_controller_mrft_pitch);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)update_controller_pid_yaw);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)update_controller_pid_yaw_rate);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)set_height_offset);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)&wait_1s);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)set_settings);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)initial_pose_waypoint);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)flight_command);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)reset_z);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)&wait_100ms);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)arm_motors);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)flight_command); //At this moment the DNN node should be run
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)reset_z);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)takeoff_relative_waypoint);
    mrft_takeoff_dnn_pipeline.addElement((FlightElement*)flight_command);

    
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
    #ifdef MRFT_TAKEOFF_DNN
    main_scenario.AddFlightPipeline(&mrft_takeoff_dnn_pipeline);
    #endif

    main_scenario.StartScenario();
    Logger::getAssignedLogger()->log("Main Done",LoggerLevel::Info);
    
    while(ros::ok){
        ros::spinOnce();
    }
}
