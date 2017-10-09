#include "MavlinkProcess.h"
#include "malloc.h"


MavlinkProcess::MavlinkProcess()
{
	sys_id = 0;
	com_id = 0;
}


MavlinkProcess::~MavlinkProcess()
{
}

void MavlinkProcess::setInspector(MavlinkInspector* in)
{
	p_inspector = in;
}


void MavlinkProcess::Parse(uint8_t* msg)
{
	uint8_t payload_length = msg[1];
	uint8_t sequence = msg[2];
	uint8_t sysid = msg[3];
	uint8_t comid = msg[4];
	uint8_t msgid = msg[5];
	uint8_t* data = (uint8_t*)malloc(payload_length);

	for (int i = 0; i < payload_length; i++)
	{
		data[i] = msg[6 + i];
	}

	p_inspector->Notify(sysid, msgid, data);
	sys_id = sysid;
	com_id = comid;
	switch (msgid)
	{
	    case MAVLINK_MSG_ID_HEARTBEAT:                                   //#0
			mavlink_heartbeat_t package_heartbeat;
			memcpy(&package_heartbeat, data, payload_length);
		break;

		case MAVLINK_MSG_ID_SYS_STATUS:                                  //#1
			mavlink_sys_status_t package_sys_status;
			memcpy(&package_sys_status, data, payload_length);
		break;

		case MAVLINK_MSG_ID_SYSTEM_TIME:                                 //#2
			mavlink_system_time_t package_system_time;
			memcpy(&package_system_time, data, payload_length);
		break;

		case MAVLINK_MSG_ID_PING:                                        //#4

		break;
	    
		case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:                     //#5

		break;

		case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK:                 //#6

		break;

		case MAVLINK_MSG_ID_AUTH_KEY:                                    //#7

		break;

		case MAVLINK_MSG_ID_SET_MODE:                                    //#11

		break;

		case MAVLINK_MSG_ID_PARAM_REQUEST_READ:                          //#20

		break;

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:                          //#21

		break;

		case MAVLINK_MSG_ID_PARAM_VALUE:                                 //#22
			mavlink_param_value_t package_param_value;
			memcpy(&package_param_value, data, payload_length);
		break;

		case MAVLINK_MSG_ID_PARAM_SET:                                   //#23

		break;

		case MAVLINK_MSG_ID_GPS_RAW_INT:                                 //#24
			mavlink_gps_raw_int_t package_gps_raw_int;
			memcpy(&package_gps_raw_int, data, payload_length);
		break;

		case MAVLINK_MSG_ID_GPS_STATUS:                                  //#25

		break;

		case MAVLINK_MSG_ID_SCALED_IMU:                                  //#26

		break;

		case MAVLINK_MSG_ID_RAW_IMU:                                     //27
			mavlink_raw_imu_t package_raw_imu;
			memcpy(&package_raw_imu, data, payload_length);
		break;

		case MAVLINK_MSG_ID_RAW_PRESSURE:                                //28

		break;

		case MAVLINK_MSG_ID_SCALED_PRESSURE:                             //29
			mavlink_scaled_pressure_t package_scaled_pressure;
			memcpy(&package_scaled_pressure, data, payload_length);
		break;

		case MAVLINK_MSG_ID_ATTITUDE:                                    //30
			mavlink_attitude_t package_attitude;
			memcpy(&package_attitude, data, payload_length);
		break;

		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:                         //31

		break;

		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:                         //33
			mavlink_global_position_int_t package_global_position_int;
			memcpy(&package_global_position_int, data, payload_length);
		break;

		case MAVLINK_MSG_ID_RC_CHANNELS_RAW:                             //35
			mavlink_rc_channels_raw_t package_rc_channels_raw;
			memcpy(&package_rc_channels_raw, data, payload_length);
		break;

		case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:                            //36
			mavlink_servo_output_raw_t package_servo_output_raw;
			memcpy(&package_servo_output_raw, data, payload_length);
		break;

		case MAVLINK_MSG_ID_MISSION_ITEM:                                //39
			mavlink_mission_item_t package_mission_item;
			memcpy(&package_mission_item, data, payload_length);
		break;

		case MAVLINK_MSG_ID_MISSION_CURRENT:                             //42
			mavlink_mission_current_t package_mission_current;
			memcpy(&package_mission_current, data, payload_length);
		break;

		case MAVLINK_MSG_ID_MISSION_COUNT:                               //44
			mavlink_mission_count_t package_mission_count;
			memcpy(&package_mission_count, data, payload_length);
		break;

		case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:                       //62
			mavlink_nav_controller_output_t package_nav_controller_output;
			memcpy(&package_nav_controller_output, data, payload_length);
		break;

		case MAVLINK_MSG_ID_RC_CHANNELS:                                 //65
			mavlink_rc_channels_t package_rc_channels;
			memcpy(&package_rc_channels, data, payload_length);
		break;

		case MAVLINK_MSG_ID_VFR_HUD:                                     //74
			mavlink_vfr_hud_t package_vfr_hud;
			memcpy(&package_vfr_hud, data, payload_length);
		break;

		case MAVLINK_MSG_ID_COMMAND_ACK:                                 //77
			mavlink_command_ack_t package_command_ack;
			memcpy(&package_command_ack, data, payload_length);
		break;

		case MAVLINK_MSG_ID_SCALED_IMU2:                                //116
			mavlink_scaled_imu2_t package_scaled_imu2;
			memcpy(&package_scaled_imu2, data, payload_length);
		break;

		case MAVLINK_MSG_ID_POWER_STATUS:                               //125
			mavlink_power_status_t package_power_status;
			memcpy(&package_power_status, data, payload_length);
		break;

		case MAVLINK_MSG_ID_AUTOPILOT_VERSION:                          //148
			mavlink_autopilot_version_t package_autopilot_version;
			memcpy(&package_autopilot_version, data, payload_length);
		break;

		case MAVLINK_MSG_ID_SENSOR_OFFSETS:                             //150
			mavlink_sensor_offsets_t package_sensor_offsets;
			memcpy(&package_sensor_offsets, data, payload_length);
		break;

		case MAVLINK_MSG_ID_MEMINFO:                                    //152
			mavlink_meminfo_t package_meminfo;
			memcpy(&package_meminfo, data, payload_length);
		break;

		case MAVLINK_MSG_ID_AHRS:                                        //163
			mavlink_ahrs_t package_ahrs;
			memcpy(&package_ahrs, data, payload_length);
		break;

		case MAVLINK_MSG_ID_HWSTATUS:                                    //165
			mavlink_hwstatus_t package_hwstatus;
			memcpy(&package_hwstatus, data, payload_length);
			break;

		case MAVLINK_MSG_ID_AHRS2:                                       //178
			mavlink_ahrs2_t package_ahrs2;
			memcpy(&package_ahrs2, data, payload_length);
			break;

		case MAVLINK_MSG_ID_AHRS3:                                       //182
			mavlink_ahrs3_t package_ahrs3;
			memcpy(&package_ahrs3, data, payload_length);
			break;

		case MAVLINK_MSG_ID_EKF_STATUS_REPORT:                           //193
			mavlink_ekf_status_report_t package_ekf_status_report;
			memcpy(&package_ekf_status_report, data, payload_length);
			break;

		case MAVLINK_MSG_ID_VIBRATION :                                  //241
			mavlink_vibration_t package_vibration;
			memcpy(&package_vibration, data, payload_length);
			break;

		case MAVLINK_MSG_ID_STATUSTEXT:                                  //253
			mavlink_statustext_t package_statustext;
			memcpy(&package_statustext, data, payload_length);
			break;
	default:
		break;
	}




}

uint8_t MavlinkProcess::Get_sys_id() {
	return sys_id;
}

uint8_t MavlinkProcess::Get_com_id() {
	return com_id;
}