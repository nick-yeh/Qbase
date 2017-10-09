#pragma once
#include "ardupilotmega/mavlink.h"
#include "common/mavlink.h"
#include "uAvionix/mavlink.h"

class MavlinkSystem
{
private:
	mavlink_heartbeat_t package_heartbeat;
	mavlink_sys_status_t package_sys_status;
	mavlink_system_time_t package_system_time;
	mavlink_param_value_t package_param_value;
	mavlink_gps_raw_int_t package_gps_raw_int;
	mavlink_raw_imu_t package_raw_imu;
	mavlink_scaled_pressure_t package_scaled_pressure;
	mavlink_attitude_t package_attitude;
	mavlink_global_position_int_t package_global_position_int;
	mavlink_rc_channels_raw_t package_rc_channels_raw;
	mavlink_servo_output_raw_t package_servo_output_raw;
	mavlink_mission_item_t package_mission_item;
	mavlink_mission_current_t package_mission_current;
	mavlink_mission_count_t package_mission_count;
	mavlink_nav_controller_output_t package_nav_controller_output;
	mavlink_rc_channels_t package_rc_channels;
	mavlink_vfr_hud_t package_vfr_hud;
	mavlink_command_ack_t package_command_ack;
	mavlink_scaled_imu2_t package_scaled_imu2;
	mavlink_power_status_t package_power_status;
	mavlink_autopilot_version_t package_autopilot_version;
	mavlink_sensor_offsets_t package_sensor_offsets;
	mavlink_meminfo_t package_meminfo;
	mavlink_ahrs_t package_ahrs;
	mavlink_hwstatus_t package_hwstatus;
	mavlink_ahrs2_t package_ahrs2;
	mavlink_ahrs3_t package_ahrs3;
	mavlink_ekf_status_report_t package_ekf_status_report;
	mavlink_vibration_t package_vibration;
	mavlink_statustext_t package_statustext;

public:
	MavlinkSystem();
	~MavlinkSystem();
};

