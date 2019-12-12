#pragma once
#include <stdint.h>

const struct tPacketProp{
	uint8_t pad_len = 3;
	uint8_t hdr_len = 2;
	uint8_t crc_len = 2;
	uint8_t pad_data[3] = { 0xFF, 0x00, 0xAA };
	uint8_t pad_EOH = 0xFF;
	uint8_t pad_EOP = 0xFF;
}PacketProp;

enum class msg_type {TESTMSG, SERIALDATA ,reference, THREEAXISSENSORMSG, VELOCITY, ACCELERATION, THERMALIMAGE, optitrack, FLIGHTSCENARIO, POSITION, ATTITUDE, HEADING, NOZZLEMSG,control_system, user, controller, float_msg, switcher, VECTOR3D, external_reference, ack,internal_msg_start,internal_ros };
enum class msg_type_optitrack {position, attitude};
enum class msg_type_flight_scenario {USER, SET_PID};
enum class control_system {roll, pitch, yaw, x, y, z, null_type};
enum class block_type {controller, provider, reference};
enum class switcher_type {controller, provider, reference, null_type};
enum class controller_type {pid, mrft};
enum class reference_type {process_variable_ref, restricted_process_variable_ref};
enum class internal_switcher_type {position_provider, attitude_provider, reference, controller};
enum class controller_msg_type {data, change_settings, command};
enum class control_system_msg_type {switch_in_out, add_block, change_PID_settings, to_system, provider_data};
enum class ack_msg_type { raw_packet, payload, nozzle, obstacle, mission,ack,internal_msg_start,internal_ros };
enum class ros_msg_type {ros_obstacle_distance,ros_aircraft_attitude};
enum class block_frequency {hz100 = 100, hz1000 = 1000, hhz1000 = 1001};
