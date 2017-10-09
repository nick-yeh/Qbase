#include "MavlinkCommand.h"



MavlinkCommand::MavlinkCommand()
{
	m_command_count = 0;
}


MavlinkCommand::~MavlinkCommand()
{
}


void MavlinkCommand::Make_Command_Long(uint8_t target_sys, uint8_t target_component, uint16_t command, uint8_t confirmation,
												float param1,
												float param2,
												float param3,
												float param4,
												float param5,
												float param6,
												float param7,
												char buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN])
{
	buf[0] = 0xFE;
	buf[1] = MAVLINK_MSG_ID_COMMAND_LONG_LEN;
	buf[2] = m_command_count;
	buf[3] = 0xFF;
	buf[4] = 0x00;
	buf[5] = MAVLINK_MSG_ID_COMMAND_LONG;

	memcpy(buf +  6, &param1, 4);
	memcpy(buf + 10, &param2, 4);
	memcpy(buf + 14, &param3, 4);
	memcpy(buf + 18, &param4, 4);
	memcpy(buf + 22, &param5, 4);
	memcpy(buf + 26, &param6, 4);
	memcpy(buf + 30, &param7, 4);
	memcpy(buf + 34, &command, 2);
	buf[36] = target_sys;
	buf[37] = target_component;
	buf[38] = confirmation;

	uint16_t crc = CRC_gengerate((uint8_t*)buf);
	buf[39] = (unsigned char)crc & 0xFF;
	buf[40] = crc >> 8;

	m_command_count++;
}



void MavlinkCommand::Make_Command_Set_Attitude_Target(	uint32_t time_boot_ms, 
														uint8_t target_sys, 
														uint8_t target_component, 
														uint8_t type_mask, float quaternion[4], 
														float body_roll_rate, 
														float body_pitch_rate, 
														float body_yaw_rate, 
														float thrust, 
														char buf[MAVLINK_MSG_PACKAGE_SET_ATTITUDE_TARGET_LEN])
{
	buf[0] = 0xFE;
	buf[1] = MAVLINK_MSG_ID_SET_ATTITUDE_TARGET_LEN;
	buf[2] = m_command_count;
	buf[3] = 0xFF;
	buf[4] = 0x00;
	buf[5] = MAVLINK_MSG_ID_SET_ATTITUDE_TARGET;

	memcpy(buf + 6, &time_boot_ms, 4);
	memcpy(buf + 10, &quaternion[0], 4);
	memcpy(buf + 14, &quaternion[1], 4);
	memcpy(buf + 18, &quaternion[2], 4);
	memcpy(buf + 22, &quaternion[3], 4);
	memcpy(buf + 26, &body_roll_rate, 4);
	memcpy(buf + 30, &body_pitch_rate, 4);
	memcpy(buf + 34, &body_yaw_rate, 4);
	memcpy(buf + 38, &thrust, 4);
	buf[42] = target_sys;
	buf[43] = target_component;
	buf[44] = type_mask;

	uint16_t crc = CRC_gengerate((uint8_t*)buf);
	buf[45] = (unsigned char)crc & 0xFF;
	buf[46] = crc >> 8;

	m_command_count++;
}

void MavlinkCommand::Make_Command_Set_Mode(	uint32_t custom_mode, 
											uint8_t base_mode, 
											uint8_t target_system, 
											char buf[MAVLINK_MSG_PACKAGE_SET_MODE_LEN])
{
	buf[0] = 0xFE;
	buf[1] = MAVLINK_MSG_ID_SET_MODE_LEN;
	buf[2] = m_command_count;
	buf[3] = 0xFF;
	buf[4] = 0x00;
	buf[5] = MAVLINK_MSG_ID_SET_MODE;

	memcpy(buf + 6, &custom_mode, 4);
	buf[10] = target_system;
	buf[11] = base_mode;

	uint16_t crc = CRC_gengerate((uint8_t*)buf);
	buf[12] = (unsigned char)crc & 0xFF;
	buf[13] = crc >> 8;

	m_command_count++;

}


void MavlinkCommand::Make_Command_Set_Position_Target_Global_Int(   uint32_t time_boot_ms, /*< Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.*/
																	int32_t lat_int, /*< X Position in WGS84 frame in 1e7 * meters*/
																	int32_t lon_int, /*< Y Position in WGS84 frame in 1e7 * meters*/
																	float alt, /*< Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT*/
																	float vx, /*< X velocity in NED frame in meter / s*/
																	float vy, /*< Y velocity in NED frame in meter / s*/
																	float vz, /*< Z velocity in NED frame in meter / s*/
																	float afx, /*< X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
																	float afy, /*< Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
																	float afz, /*< Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
																	float yaw, /*< yaw setpoint in rad*/
																	float yaw_rate, /*< yaw rate setpoint in rad/s*/
																	uint16_t type_mask, /*< Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate*/
																	uint8_t target_system, /*< System ID*/
																	uint8_t target_component, /*< Component ID*/
																	uint8_t coordinate_frame, /*< Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11*/
																	char buf[MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN])
{
	buf[0] = 0xFE;
	buf[1] = MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN;
	buf[2] = m_command_count;
	buf[3] = 0xFF;
	buf[4] = 0x00;
	buf[5] = MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT;

	memcpy(buf + 6, &time_boot_ms, 4);
	memcpy(buf + 10, &lat_int, 4);
	memcpy(buf + 14, &lon_int, 4);
	memcpy(buf + 18, &alt, 4);
	memcpy(buf + 22, &vx, 4);
	memcpy(buf + 26, &vy, 4);
	memcpy(buf + 30, &vz, 4);
	memcpy(buf + 34, &afx, 4);
	memcpy(buf + 38, &afy, 4);
	memcpy(buf + 42, &afz, 4);
	memcpy(buf + 46, &yaw, 4);
	memcpy(buf + 50, &yaw_rate, 4);
	memcpy(buf + 54, &type_mask, 2);
	buf[56] = target_system;
	buf[57] = target_component;
	buf[58] = coordinate_frame;


	uint16_t crc = CRC_gengerate((uint8_t*)buf);
	buf[59] = (unsigned char)crc & 0xFF;
	buf[60] = crc >> 8;

	m_command_count++;

}
