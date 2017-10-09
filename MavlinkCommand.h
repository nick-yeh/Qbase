#pragma once
#include <QString>
#include "ui_MavlinkInspector.h"
#include "ardupilotmega/mavlink.h"
#include "common/mavlink.h"
#include "uAvionix/mavlink.h"
#include "checkcrc.h"

#define MAVLINK_MSG_PACKAGE_SET_ATTITUDE_TARGET_LEN MAVLINK_MSG_ID_SET_ATTITUDE_TARGET_LEN + 8
#define MAVLINK_MSG_PACKAGE_COMMAND_LONG_LEN MAVLINK_MSG_ID_COMMAND_LONG_LEN + 8
#define MAVLINK_MSG_PACKAGE_SET_MODE_LEN MAVLINK_MSG_ID_SET_MODE_LEN + 8
#define MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN +8

#define TYPE_MASK_POSITION 0b111111111000
#define TYPE_MASK_VELOCITY 0b111111000111
#define TARGET_FRAME 6

typedef struct __mavlink_set_mode_estimate_t
{
	uint32_t custom_mode;
	uint8_t target_system;
	uint8_t base_mode;

} mavlink_set_mode_estimate_t;

class MavlinkCommand
{
public:
	MavlinkCommand();
	~MavlinkCommand();


public:
	uint8_t m_command_count;

	void Make_Command_Long(uint8_t target_sys, uint8_t target_component, uint16_t command, uint8_t confirmation,
		float param1,
		float param2,
		float param3,
		float param4,
		float param5,
		float param6,
		float param7,
		char buf[MAVLINK_MSG_ID_COMMAND_LONG_LEN]);

	void Make_Command_Set_Attitude_Target(uint32_t time_boot_ms, uint8_t target_sys, uint8_t target_component, uint8_t type_mask, float quaternion[4], float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust, char buf[MAVLINK_MSG_PACKAGE_SET_ATTITUDE_TARGET_LEN]);
	void Make_Command_Set_Mode(uint32_t custom_mode, uint8_t base_mode, uint8_t target_system,char buf[MAVLINK_MSG_PACKAGE_SET_MODE_LEN]);
	void Make_Command_Set_Position_Target_Global_Int(   uint32_t time_boot_ms, /*< Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.*/
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
														char buf[MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN]);



	enum MAV_CMD : ushort
	{
		///<summary> Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing). NaN for unchanged.| Latitude| Longitude| Altitude|  </summary>
		WAYPOINT = 16,
		///<summary> Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  </summary>
		LOITER_UNLIM = 17,
		///<summary> Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  </summary>
		LOITER_TURNS = 18,
		///<summary> Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  </summary>
		LOITER_TIME = 19,
		///<summary> Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		RETURN_TO_LAUNCH = 20,
		///<summary> Land at location |Abort Alt| Empty| Empty| Desired yaw angle. NaN for unchanged.| Latitude| Longitude| Altitude|  </summary>
		LAND = 21,
		///<summary> Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.| Latitude| Longitude| Altitude|  </summary>
		TAKEOFF = 22,
		///<summary> Land at local position (local frame only) |Landing target number (if available)| Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land| Landing descend rate [ms^-1]| Desired yaw angle [rad]| Y-axis position [m]| X-axis position [m]| Z-axis / ground level position [m]|  </summary>
		LAND_LOCAL = 23,
		///<summary> Takeoff from local position (local frame only) |Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]| Empty| Takeoff ascend rate [ms^-1]| Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these| Y-axis position [m]| X-axis position [m]| Z-axis position [m]|  </summary>
		TAKEOFF_LOCAL = 24,
		///<summary> Vehicle following, i.e. this waypoint represents the position of a moving vehicle |Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation| Ground speed of vehicle to be followed| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  </summary>
		FOLLOW = 25,
		///<summary> Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude. | Empty| Empty| Empty| Empty| Empty| Desired altitude in meters|  </summary>
		CONTINUE_AND_CHANGE_ALT = 30,
		///<summary> Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint.  |Heading Required (0 = False)| Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.| Empty| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location| Latitude| Longitude| Altitude|  </summary>
		LOITER_TO_ALT = 31,
		///<summary> Being following a target |System ID (the system ID of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode| RESERVED| RESERVED| altitude flag: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home| altitude| RESERVED| TTL in seconds in which the MAV should go to the default position hold mode after a message rx timeout|  </summary>
		DO_FOLLOW = 32,
		///<summary> Reposition the MAV after a follow target command has been sent |Camera q1 (where 0 is on the ray from the camera to the tracking device)| Camera q2| Camera q3| Camera q4| altitude offset from target (m)| X offset from target (m)| Y offset from target (m)|  </summary>
		DO_FOLLOW_REPOSITION = 33,
		///<summary> Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  </summary>
		ROI = 80,
		///<summary> Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  </summary>
		PATHPLANNING = 81,
		///<summary> Navigate to MISSION using a spline path. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Empty| Empty| Empty| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  </summary>
		SPLINE_WAYPOINT = 82,
		///<summary> Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up. |altitude (m)| descent speed (m/s)| Wiggle Time (s)| Empty| Empty| Empty| Empty|  </summary>
		ALTITUDE_WAIT = 83,
		///<summary> Takeoff from ground using VTOL mode |Empty| Empty| Empty| Yaw angle in degrees. NaN for unchanged.| Latitude| Longitude| Altitude|  </summary>
		VTOL_TAKEOFF = 84,
		///<summary> Land using VTOL mode |Empty| Empty| Empty| Yaw angle in degrees. NaN for unchanged.| Latitude| Longitude| Altitude|  </summary>
		VTOL_LAND = 85,
		///<summary> hand control over to an external controller |On / Off (> 0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		GUIDED_ENABLE = 92,
		///<summary> Delay the next navigation command a number of seconds or until a specified time |Delay in seconds (decimal, -1 to enable time-of-day fields)| hour (24h format, UTC, -1 to ignore)| minute (24h format, UTC, -1 to ignore)| second (24h format, UTC)| Empty| Empty| Empty|  </summary>
		DELAY = 93,
		///<summary> Descend and place payload.  Vehicle descends until it detects a hanging payload has reached the ground, the gripper is opened to release the payload |Maximum distance to descend (meters)| Empty| Empty| Empty| Latitude (deg * 1E7)| Longitude (deg * 1E7)| Altitude (meters)|  </summary>
		PAYLOAD_PLACE = 94,
		///<summary> NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		LAST = 95,
		///<summary> Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		CONDITION_DELAY = 112,
		///<summary> Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  </summary>
		CONDITION_CHANGE_ALT = 113,
		///<summary> Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		CONDITION_DISTANCE = 114,
		///<summary> Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  </summary>
		CONDITION_YAW = 115,
		///<summary> NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		CONDITION_LAST = 159,
		///<summary> Set system mode. |Mode, as defined by ENUM MAV_MODE| Custom mode - this is system specific, please refer to the individual autopilot specifications for details.| Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.| Empty| Empty| Empty| Empty|  </summary>
		DO_SET_MODE = 176,
		///<summary> Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_JUMP = 177,
		///<summary> Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| absolute or relative [0,1]| Empty| Empty| Empty|  </summary>
		DO_CHANGE_SPEED = 178,
		///<summary> Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  </summary>
		DO_SET_HOME = 179,
		///<summary> Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_SET_PARAMETER = 180,
		///<summary> Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_SET_RELAY = 181,
		///<summary> Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  </summary>
		DO_REPEAT_RELAY = 182,
		///<summary> Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_SET_SERVO = 183,
		///<summary> Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  </summary>
		DO_REPEAT_SERVO = 184,
		///<summary> Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_FLIGHTTERMINATION = 185,
		///<summary> Change altitude set point. |Altitude in meters| Mav frame of new altitude (see MAV_FRAME)| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_CHANGE_ALTITUDE = 186,
		///<summary> Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0/0 if not needed. If specified then it will be used to help find the closest landing sequence. |Empty| Empty| Empty| Empty| Latitude| Longitude| Empty|  </summary>
		DO_LAND_START = 189,
		///<summary> Mission command to perform a landing from a rally point. |Break altitude (meters)| Landing speed (m/s)| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_RALLY_LAND = 190,
		///<summary> Mission command to safely abort an autonmous landing. |Altitude (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_GO_AROUND = 191,
		///<summary> Reposition the vehicle to a specific WGS84 global position. |Ground speed, less than 0 (-1) for default| Bitmask of option flags, see the MAV_DO_REPOSITION_FLAGS enum.| Reserved| Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)| Latitude (deg * 1E7)| Longitude (deg * 1E7)| Altitude (meters)|  </summary>
		DO_REPOSITION = 192,
		///<summary> If in a GPS controlled position mode, hold the current position or continue. |0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
		DO_PAUSE_CONTINUE = 193,
		///<summary> Set moving direction to forward or reverse. |Direction (0=Forward, 1=Reverse)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_SET_REVERSE = 194,
		///<summary> Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  </summary>
		DO_CONTROL_VIDEO = 200,
		///<summary> Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  </summary>
		DO_SET_ROI = 201,
		///<summary> Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|  </summary>
		DO_DIGICAM_CONFIGURE = 202,
		///<summary> Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|  </summary>
		DO_DIGICAM_CONTROL = 203,
		///<summary> Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|  </summary>
		DO_MOUNT_CONFIGURE = 204,
		///<summary> Mission command to control a camera or antenna mount |pitch (WIP: DEPRECATED: or lat in degrees) depending on mount mode.| roll (WIP: DEPRECATED: or lon in degrees) depending on mount mode.| yaw (WIP: DEPRECATED: or alt in meters) depending on mount mode.| WIP: alt in meters depending on mount mode.| WIP: latitude in degrees * 1E7, set if appropriate mount mode.| WIP: longitude in degrees * 1E7, set if appropriate mount mode.| MAV_MOUNT_MODE enum value|  </summary>
		DO_MOUNT_CONTROL = 205,
		///<summary> Mission command to set CAM_TRIGG_DIST for this flight |Camera trigger distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_SET_CAM_TRIGG_DIST = 206,
		///<summary> Mission command to enable the geofence |enable? (0=disable, 1=enable, 2=disable_floor_only)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_FENCE_ENABLE = 207,
		///<summary> Mission command to trigger a parachute |action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_PARACHUTE = 208,
		///<summary> Mission command to perform motor test |motor sequence number (a number from 1 to max number of motors on the vehicle)| throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)| throttle| timeout (in seconds)| Empty| Empty| Empty|  </summary>
		DO_MOTOR_TEST = 209,
		///<summary> Change to/from inverted flight |inverted (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_INVERTED_FLIGHT = 210,
		///<summary> Mission command to operate EPM gripper |gripper number (a number from 1 to max number of grippers on the vehicle)| gripper action (0=release, 1=grab. See GRIPPER_ACTIONS enum)| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_GRIPPER = 211,
		///<summary> Enable/disable autotune |enable (1: enable, 0:disable)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_AUTOTUNE_ENABLE = 212,
		///<summary> Sets a desired vehicle turn angle and speed change |yaw angle to adjust steering by in centidegress| speed - normalized to 0 .. 1| Empty| Empty| Empty| Empty| Empty|  </summary>
		SET_YAW_SPEED = 213,
		///<summary> Mission command to control a camera or antenna mount, using a quaternion as reference. |q1 - quaternion param #1, w (1 in null-rotation)| q2 - quaternion param #2, x (0 in null-rotation)| q3 - quaternion param #3, y (0 in null-rotation)| q4 - quaternion param #4, z (0 in null-rotation)| Empty| Empty| Empty|  </summary>
		DO_MOUNT_CONTROL_QUAT = 220,
		///<summary> set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_GUIDED_MASTER = 221,
		///<summary> set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty|  </summary>
		DO_GUIDED_LIMITS = 222,
		///<summary> Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines |0: Stop engine, 1:Start Engine| 0: Warm start, 1:Cold start. Controls use of choke where applicable| Height delay (meters). This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_ENGINE_CONTROL = 223,
		///<summary> NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_LAST = 240,
		///<summary> Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero. |1: gyro calibration, 3: gyro temperature calibration| 1: magnetometer calibration| 1: ground pressure calibration| 1: radio RC calibration, 2: RC trim calibration| 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration| 1: APM: compass/motor interference calibration / PX4: airspeed calibration| 1: ESC calibration, 3: barometer temperature calibration|  </summary>
		PREFLIGHT_CALIBRATION = 241,
		///<summary> Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  </summary>
		PREFLIGHT_SET_SENSOR_OFFSETS = 242,
		///<summary> Trigger UAVCAN config. This command will be only accepted if in pre-flight mode. |1: Trigger actuator ID assignment and direction mapping.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
		PREFLIGHT_UAVCAN = 243,
		///<summary> Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz logging)| Reserved| Empty| Empty| Empty|  </summary>
		PREFLIGHT_STORAGE = 245,
		///<summary> Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.| WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded| WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded| Reserved, send 0| Reserved, send 0| WIP: ID (e.g. camera ID -1 for all IDs)|  </summary>
		PREFLIGHT_REBOOT_SHUTDOWN = 246,
		///<summary> Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  </summary>
		OVERRIDE_GOTO = 252,
		///<summary> start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  </summary>
		MISSION_START = 300,
		///<summary> Arms / Disarms a component |1 to arm, 0 to disarm|  </summary>
		COMPONENT_ARM_DISARM = 400,
		///<summary> Request the home position from the vehicle. |Reserved| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
		GET_HOME_POSITION = 410,
		///<summary> Starts receiver pairing |0:Spektrum| 0:Spektrum DSM2, 1:Spektrum DSMX|  </summary>
		START_RX_PAIR = 500,
		///<summary> Request the interval between messages for a particular MAVLink message ID |The MAVLink message ID|  </summary>
		GET_MESSAGE_INTERVAL = 510,
		///<summary> Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM |The MAVLink message ID| The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate.|  </summary>
		SET_MESSAGE_INTERVAL = 511,
		///<summary> Request autopilot capabilities |1: Request autopilot version| Reserved (all remaining params)|  </summary>
		REQUEST_AUTOPILOT_CAPABILITIES = 520,
		///<summary> WIP: Request camera information (CAMERA_INFORMATION) |1: Request camera capabilities| Camera ID| Reserved (all remaining params)|  </summary>
		REQUEST_CAMERA_INFORMATION = 521,
		///<summary> WIP: Request camera settings (CAMERA_SETTINGS) |1: Request camera settings| Camera ID| Reserved (all remaining params)|  </summary>
		REQUEST_CAMERA_SETTINGS = 522,
		///<summary> WIP: Set the camera settings part 1 (CAMERA_SETTINGS) |Camera ID| Aperture (1/value)| Aperture locked (0: auto, 1: locked)| Shutter speed in s| Shutter speed locked (0: auto, 1: locked)| ISO sensitivity| ISO sensitivity locked (0: auto, 1: locked)|  </summary>
		SET_CAMERA_SETTINGS_1 = 523,
		///<summary> WIP: Set the camera settings part 2 (CAMERA_SETTINGS) |Camera ID| White balance locked (0: auto, 1: locked)| White balance (color temperature in K)| Reserved for camera mode ID| Reserved for color mode ID| Reserved for image format ID| Reserved|  </summary>
		SET_CAMERA_SETTINGS_2 = 524,
		///<summary> WIP: Request storage information (STORAGE_INFORMATION) |1: Request storage information| Storage ID| Reserved (all remaining params)|  </summary>
		REQUEST_STORAGE_INFORMATION = 525,
		///<summary> WIP: Format a storage medium |1: Format storage| Storage ID| Reserved (all remaining params)|  </summary>
		STORAGE_FORMAT = 526,
		///<summary> WIP: Request camera capture status (CAMERA_CAPTURE_STATUS) |1: Request camera capture status| Camera ID| Reserved (all remaining params)|  </summary>
		REQUEST_CAMERA_CAPTURE_STATUS = 527,
		///<summary> WIP: Request flight information (FLIGHT_INFORMATION) |1: Request flight information| Reserved (all remaining params)|  </summary>
		REQUEST_FLIGHT_INFORMATION = 528,
		///<summary> Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. |Duration between two consecutive pictures (in seconds)| Number of images to capture total - 0 for unlimited capture| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc), set to 0 if param 4/5 are used, set to -1 for highest resolution possible.| WIP: Resolution horizontal in pixels| WIP: Resolution horizontal in pixels| WIP: Camera ID|  </summary>
		IMAGE_START_CAPTURE = 2000,
		///<summary> Stop image capture sequence |Camera ID| Reserved|  </summary>
		IMAGE_STOP_CAPTURE = 2001,
		///<summary> Enable or disable on-board camera triggering system. |Trigger enable/disable (0 for disable, 1 for start)| Shutter integration time (in ms)| Reserved|  </summary>
		DO_TRIGGER_CONTROL = 2003,
		///<summary> Starts video capture (recording) |Camera ID (0 for all cameras), 1 for first, 2 for second, etc.| Frames per second, set to -1 for highest framerate possible.| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc), set to 0 if param 4/5 are used, set to -1 for highest resolution possible.| WIP: Resolution horizontal in pixels| WIP: Resolution horizontal in pixels| WIP: Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise time in Hz)|  </summary>
		VIDEO_START_CAPTURE = 2500,
		///<summary> Stop the current video capture (recording) |WIP: Camera ID| Reserved|  </summary>
		VIDEO_STOP_CAPTURE = 2501,
		///<summary> Request to start streaming logging data over MAVLink (see also LOGGING_DATA message) |Format: 0: ULog| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  </summary>
		LOGGING_START = 2510,
		///<summary> Request to stop streaming log data over MAVLink |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  </summary>
		LOGGING_STOP = 2511,
		///<summary>  |Landing gear ID (default: 0, -1 for all)| Landing gear position (Down: 0, Up: 1, NAN for no change)| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN|  </summary>
		AIRFRAME_CONFIGURATION = 2520,
		///<summary> Create a panorama at the current position |Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)| Viewing angle vertical of panorama (in degrees)| Speed of the horizontal rotation (in degrees per second)| Speed of the vertical rotation (in degrees per second)|  </summary>
		PANORAMA_CREATE = 2800,
		///<summary> Request VTOL transition |The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.|  </summary>
		DO_VTOL_TRANSITION = 3000,
		///<summary> This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocites along all three axes.                    | </summary>
		SET_GUIDED_SUBMODE_STANDARD = 4000,
		///<summary> This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.                    |Radius of desired circle in CIRCLE_MODE| User defined| User defined| User defined| Unscaled target latitude of center of circle in CIRCLE_MODE| Unscaled target longitude of center of circle in CIRCLE_MODE|  </summary>
		SET_GUIDED_SUBMODE_CIRCLE = 4001,
		///<summary> Fence return point. There can only be one fence return point.          |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  </summary>
		FENCE_RETURN_POINT = 5000,
		///<summary> Fence vertex for an inclusion polygon. The vehicle must stay within this area. Minimum of 3 vertices required.          |Polygon vertex count| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  </summary>
		FENCE_POLYGON_VERTEX_INCLUSION = 5001,
		///<summary> Fence vertex for an exclusion polygon. The vehicle must stay outside this area. Minimum of 3 vertices required.          |Polygon vertex count| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  </summary>
		FENCE_POLYGON_VERTEX_EXCLUSION = 5002,
		///<summary> Rally point. You can have multiple rally points defined.          |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  </summary>
		RALLY_POINT = 5100,
		///<summary> Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.| Desired ground speed at release time. This can be overriden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.| Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Altitude, in meters AMSL|  </summary>
		PAYLOAD_PREPARE_DEPLOY = 30001,
		///<summary> Control the payload deployment. |Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
		PAYLOAD_CONTROL_DEPLOY = 30002,
		///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
		WAYPOINT_USER_1 = 31000,
		///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
		WAYPOINT_USER_2 = 31001,
		///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
		WAYPOINT_USER_3 = 31002,
		///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
		WAYPOINT_USER_4 = 31003,
		///<summary> User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
		WAYPOINT_USER_5 = 31004,
		///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
		SPATIAL_USER_1 = 31005,
		///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
		SPATIAL_USER_2 = 31006,
		///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
		SPATIAL_USER_3 = 31007,
		///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
		SPATIAL_USER_4 = 31008,
		///<summary> User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  </summary>
		SPATIAL_USER_5 = 31009,
		///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
		USER_1 = 31010,
		///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
		USER_2 = 31011,
		///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
		USER_3 = 31012,
		///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
		USER_4 = 31013,
		///<summary> User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  </summary>
		USER_5 = 31014,
		///<summary> A system wide power-off event has been initiated. |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		POWER_OFF_INITIATED = 42000,
		///<summary> FLY button has been clicked. |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		SOLO_BTN_FLY_CLICK = 42001,
		///<summary> FLY button has been held for 1.5 seconds. |Takeoff altitude| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		SOLO_BTN_FLY_HOLD = 42002,
		///<summary> PAUSE button has been clicked. |1 if Solo is in a shot mode, 0 otherwise| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		SOLO_BTN_PAUSE_CLICK = 42003,
		///<summary> Initiate a magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Automatically retry on failure (0=no retry, 1=retry).| Save without user input (0=require input, 1=autosave).| Delay (seconds)| Autoreboot (0=user reboot, 1=autoreboot)| Empty| Empty|  </summary>
		DO_START_MAG_CAL = 42424,
		///<summary> Initiate a magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_ACCEPT_MAG_CAL = 42425,
		///<summary> Cancel a running magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_CANCEL_MAG_CAL = 42426,
		///<summary> Command autopilot to get into factory test/diagnostic mode |0 means get out of test mode, 1 means get into test mode| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		SET_FACTORY_TEST_MODE = 42427,
		///<summary> Reply with the version banner |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		DO_SEND_BANNER = 42428,
		///<summary> Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the vehicle in. When sent to the vehicle says what position the vehicle is in. |Position, one of the ACCELCAL_VEHICLE_POS enum values| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		ACCELCAL_VEHICLE_POS = 42429,
		///<summary> Causes the gimbal to reset and boot as if it was just powered on |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		GIMBAL_RESET = 42501,
		///<summary> Reports progress and success or failure of gimbal axis calibration procedure |Gimbal axis we're reporting calibration progress for| Current calibration progress for this axis, 0x64=100%| Status of the calibration| Empty| Empty| Empty| Empty|  </summary>
		GIMBAL_AXIS_CALIBRATION_STATUS = 42502,
		///<summary> Starts commutation calibration on the gimbal |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
		GIMBAL_REQUEST_AXIS_CALIBRATION = 42503,
		///<summary> Erases gimbal application and parameters |Magic number| Magic number| Magic number| Magic number| Magic number| Magic number| Magic number|  </summary>
		GIMBAL_FULL_RESET = 42505,
	};

};

