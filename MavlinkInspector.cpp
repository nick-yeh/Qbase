#include "MavlinkInspector.h"

MavlinkInspector::MavlinkInspector(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	ui.treeWidget->setWindowTitle("Mavlink Messages");
	ui.treeWidget->setHeaderLabels(QStringList() << "Name" << "Value");
	ui.treeWidget->setColumnWidth(0, 400);  //…Ë÷√¡–øÌ

	time.start();
	for (int i = 0; i < 256; i++) Interval[i] = 0;

	boot_time_ms = 0;
    
	//setWindowFlags(this->windowFlags() | Qt::WindowStaysOnTopHint);

}

DroneInfo MavlinkInspector::Get_drone_info(QString sysname)
{
	for (int i = 0; i < ui.treeWidget->topLevelItemCount(); i++)
	{
		if (ui.treeWidget->topLevelItem(i)->text(0) == sysname)
		{
			QTreeWidgetItem *current = ui.treeWidget->topLevelItem(i);
			for (int j = 0; j < current->childCount(); j++)
			{
					if (current->child(j)->text(0).contains("GLOBAL POSITION INT"))
					{
						QTreeWidgetItem *package = current->child(j);
						DroneInfo info;
						if (package->childCount() != 9) break;
						info.altitude = package->child(0)->text(1).toInt();
						info.heading = package->child(1)->text(1).toInt();
						info.latitude = package->child(2)->text(1).toInt()/10000000.0;
						info.longitude = package->child(3)->text(1).toInt()/10000000.0;
						info.relative_alt = package->child(4)->text(1).toInt();
						info.timestamp = package->child(5)->text(1).toLong();
						info.vx = package->child(6)->text(1).toInt();
						info.vy = package->child(7)->text(1).toInt();
						info.vz = package->child(8)->text(1).toInt();
						return info;
						
					}
			}
		}

	}

	return DroneInfo();
}


QVector<QString> MavlinkInspector::Get_id_list() {
	QVector<QString> id_list;
	for (int i = 0; i < ui.treeWidget->topLevelItemCount(); i++)
	{
		if (ui.treeWidget->topLevelItem(i)->text(0).contains("vehicle"))
		{
			id_list.append(ui.treeWidget->topLevelItem(i)->text(0));
		}
	}
	return id_list;
}




MavlinkInspector::~MavlinkInspector()
{
}

void MavlinkInspector::Notify(uint8_t sysid, uint8_t msgid, uint8_t*data)
{	
	bool vehicle_existed = false;

	for(int i = 0; i< ui.treeWidget->topLevelItemCount();i++) {
		if (ui.treeWidget->topLevelItem(i)->text(0) == "vehicle" + QString::number(sysid))
		{   
			vehicle_existed = true;

			QTreeWidgetItem *current = ui.treeWidget->topLevelItem(i);
			bool message_existed = false;

			for (int j = 0; j < current->childCount();j++) {
				
				QString temp = current->child(j)->text(0);
				QStringList list = temp.split('#');
				if (list.length() >= 2)
				{
					if (list[0].toInt() == msgid)
					{
						message_existed = true;
						QString title = QString::number(msgid) + " #" + Translate_id(msgid);
						title = title + "(" + QString::number(GetFrequency(msgid),'f', 1) + "Hz)";
						ui.treeWidget->topLevelItem(i)->child(j)->setText(0,title);
	
						QVector<QStringList> keys = Translate_data(msgid, data);
						if (current->child(j)->childCount() == 0)
						{
							for (int k = 0; k < keys.length(); k++)
							{
								QTreeWidgetItem *child;
								child = new QTreeWidgetItem(keys[k]);
								current->child(j)->addChild(child);
							}
						}
						for (int l = 0; l < keys.length(); l++) {
							current->child(j)->child(l)->setText(1,keys[l][1]);
						}
					
					}
				}
			}
			if (!message_existed)
			{
				QString title = QString::number(msgid) + " #" + Translate_id(msgid);
				title = title + "(" +QString::number(GetFrequency(msgid), 'f', 1)+ "Hz)";
				QTreeWidgetItem* new_msg = new QTreeWidgetItem(QStringList() << title);
				QVector<QStringList> keys = Translate_data(msgid,data);
				for (int i = 0; i < keys.length(); i++)
				{
					QTreeWidgetItem *child;
					child = new QTreeWidgetItem(keys[i]);
					new_msg->addChild(child);
				}
				current->addChild(new_msg);
			}

		}
	}
	if (!vehicle_existed)
	{
		QTreeWidgetItem* new_vehicle = new QTreeWidgetItem(QStringList() << ("vehicle" + QString::number(sysid)));
		new_vehicle->setIcon(0, QIcon("images/system_icon.png"));
		QTreeWidgetItem* new_msg = new QTreeWidgetItem(QStringList() << QString::number(msgid) + " #" + Translate_id(msgid));
		new_vehicle->addChild(new_msg);

		ui.treeWidget->addTopLevelItem(new_vehicle);
	}
}

QString MavlinkInspector::Translate_id(uint8_t msgid)
{
	switch (msgid)
	{
	    case 0: return "HEARTBEAT"; break;                        
		case 1: return "SYS STATUS"; break;
		case 2: return "SYSTEM TIME"; break;
		case 4: return "PING"; break;
		case 5: return "CHANGE OPERATOR CONTROL"; break;
		case 6: return "CHANGE OPERATOR CONTROL ACK"; break;
		case 7: return "AUTH KEY"; break;
		case 11: return "SET MODE"; break;
		case 20: return "PARAM REQUEST READ"; break;
		case 21: return "PARAM REQUEST LIST"; break;
		case 22: return "PARAM VALUE"; break;
		case 23: return "PARAM SET"; break;
		case 24: return "GPS RAW INT"; break;
		case 25: return "GPS STATUS"; break;
		case 26: return "SCALED IMU"; break;
		case 27: return "RAW IMU"; break;
		case 28: return "RAW PRESSURE"; break;
		case 29: return "SCALED PRESSURE"; break;
		case 30: return "ATTITUDE"; break;
		case 31: return "ATTITUDE QUATERNION"; break;
		case 33: return "GLOBAL POSITION INT"; break;
		case 35: return "RC CHANNELS RAW"; break;
		case 36: return "SERVO OUTPUT RAW"; break;
		case 39: return "MISSION ITEM"; break;
		case 42: return "MISSION CURRENT"; break;
		case 44: return "MISSION COUNT"; break;
		case 62: return "NAV CONTROLLER OUTPUT"; break;
		case 65: return "RC CHANNELS"; break;
		case 74: return "VFR HUD"; break;
		case 77: return "COMMAND ACK"; break;
		case 116: return "SCALED IMU2"; break;
		case 125: return "POWER STATUS"; break;
		case 148: return "AUTOPILOT VERSION"; break;
		case 150: return "SENSOR OFFSETS"; break;
		case 152: return "MEMINFO"; break;
		case 163: return "AHRS"; break;
		case 165: return "HWSTATUS"; break;
		case 178: return "AHRS2"; break;
		case 182: return "AHRS3"; break;
		case 193: return "EKF STATUS REPORT"; break;
		case 241: return "VIBRATION"; break;
		case 253: return "STATUSTEXT"; break;

		default: return "UNEXPLICTED MESSAGE";
		break;
	}
}

QVector<QStringList> MavlinkInspector::Translate_data(uint8_t msgid, uint8_t* data)
{
	
	QVector<QStringList> detail;
	QStringList key;
	switch (msgid)
	{
	case MAVLINK_MSG_ID_HEARTBEAT:                                   //#0
		mavlink_heartbeat_t package_heartbeat;
		memcpy(&package_heartbeat, data, MAVLINK_MSG_ID_HEARTBEAT_LEN);
		key << "autopilot" << QString::number(package_heartbeat.autopilot);
		detail.append(key);  key.clear();
		key << "base_mode" << QString::number(package_heartbeat.base_mode);
		detail.append(key);  key.clear();
		key << "custom_mode" << QString::number(package_heartbeat.custom_mode);
		detail.append(key);  key.clear();
		key << "mavlink_version" << QString::number(package_heartbeat.mavlink_version);
		detail.append(key);  key.clear();
		key << "system_status" << QString::number(package_heartbeat.system_status);
		detail.append(key);  key.clear();
		key << "type" << QString::number(package_heartbeat.type);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_SYS_STATUS:                                  //#1
		mavlink_sys_status_t package_sys_status;
		memcpy(&package_sys_status, data, MAVLINK_MSG_ID_SYS_STATUS_LEN);
		key << "battery_remaining" << QString::number(package_sys_status.battery_remaining);
		detail.append(key);  key.clear();
		key << "current_battery" << QString::number(package_sys_status.current_battery);
		detail.append(key);  key.clear();
		key << "drop_rate_comm" << QString::number(package_sys_status.drop_rate_comm);
		detail.append(key);  key.clear();
		key << "errors_comm" << QString::number(package_sys_status.errors_comm);
		detail.append(key);  key.clear();
		key << "errors_count1" << QString::number(package_sys_status.errors_count1);
		detail.append(key);  key.clear();
		key << "errors_count2" << QString::number(package_sys_status.errors_count2);
		detail.append(key);  key.clear();
		key << "errors_count3" << QString::number(package_sys_status.errors_count3);
		detail.append(key);  key.clear();
		key << "errors_count4" << QString::number(package_sys_status.errors_count4);
		detail.append(key);  key.clear();
		key << "load" << QString::number(package_sys_status.load);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_SYSTEM_TIME:                                 //#2
		mavlink_system_time_t package_system_time;
		memcpy(&package_system_time, data, MAVLINK_MSG_ID_SYSTEM_TIME_LEN);
		key << "time_boot_ms" << QString::number(package_system_time.time_boot_ms);
		detail.append(key);  key.clear();
		key << "time_unix_usec" << QString::number(package_system_time.time_unix_usec);
		detail.append(key);  key.clear();
		return detail;
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
		memcpy(&package_param_value, data, MAVLINK_MSG_ID_PARAM_VALUE_LEN);
		key << "param_count" << QString::number(package_param_value.param_count);
		detail.append(key);  key.clear();
		key << "param_id" << QString(package_param_value.param_id);
		detail.append(key);  key.clear();
		key << "param_index" << QString::number(package_param_value.param_index);
		detail.append(key);  key.clear();
		key << "param_type" << QString::number(package_param_value.param_type);
		detail.append(key);  key.clear();
		key << "param_value" << QString::number(package_param_value.param_value);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_PARAM_SET:                                   //#23

		break;

	case MAVLINK_MSG_ID_GPS_RAW_INT:                                 //#24
		mavlink_gps_raw_int_t package_gps_raw_int;
		memcpy(&package_gps_raw_int, data, MAVLINK_MSG_ID_GPS_RAW_INT_LEN);
		key << "alt" << QString::number(package_gps_raw_int.alt);
		detail.append(key);  key.clear();
		key << "cog" << QString::number(package_gps_raw_int.cog);
		detail.append(key);  key.clear();
		key << "eph" << QString::number(package_gps_raw_int.eph);
		detail.append(key);  key.clear();
		key << "epv" << QString::number(package_gps_raw_int.epv);
		detail.append(key);  key.clear();
		key << "fix_type" << QString::number(package_gps_raw_int.fix_type);
		detail.append(key);  key.clear();
		key << "lat" << QString::number(package_gps_raw_int.lat);
		detail.append(key);  key.clear();
		key << "lon" << QString::number(package_gps_raw_int.lon,10);
		detail.append(key);  key.clear();
		key << "satellites_visible" << QString::number(package_gps_raw_int.satellites_visible);
		detail.append(key);  key.clear();
		key << "time_usec" << QString::number(package_gps_raw_int.time_usec);
		detail.append(key);  key.clear();
		key << "vel" << QString::number(package_gps_raw_int.vel);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_GPS_STATUS:                                  //#25

		break;

	case MAVLINK_MSG_ID_SCALED_IMU:                                  //#26

		break;

	case MAVLINK_MSG_ID_RAW_IMU:                                     //27
		mavlink_raw_imu_t package_raw_imu;
		memcpy(&package_raw_imu, data, MAVLINK_MSG_ID_RAW_IMU_LEN);
		key << "time_usec" << QString::number(package_raw_imu.time_usec);
		detail.append(key);  key.clear();
		key << "xacc" << QString::number(package_raw_imu.xacc);
		detail.append(key);  key.clear();
		key << "xgyro" << QString::number(package_raw_imu.xgyro);
		detail.append(key);  key.clear();
		key << "xmag" << QString::number(package_raw_imu.xmag);
		detail.append(key);  key.clear();
		key << "yacc" << QString::number(package_raw_imu.yacc);
		detail.append(key);  key.clear();
		key << "ygyro" << QString::number(package_raw_imu.ygyro);
		detail.append(key);  key.clear();
		key << "ymag" << QString::number(package_raw_imu.ymag);
		detail.append(key);  key.clear();
		key << "zacc" << QString::number(package_raw_imu.zacc);
		detail.append(key);  key.clear();
		key << "zgyro" << QString::number(package_raw_imu.zgyro);
		detail.append(key);  key.clear();
		key << "zmag" << QString::number(package_raw_imu.zmag);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_RAW_PRESSURE:                                //28

		break;

	case MAVLINK_MSG_ID_SCALED_PRESSURE:                             //29
		mavlink_scaled_pressure_t package_scaled_pressure;
		memcpy(&package_scaled_pressure, data, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN);
		key << "press_abs" << QString::number(package_scaled_pressure.press_abs);
		detail.append(key);  key.clear();
		key << "press_diff" << QString::number(package_scaled_pressure.press_diff);
		detail.append(key);  key.clear();
		key << "temperature" << QString::number(package_scaled_pressure.temperature);
		detail.append(key);  key.clear();
		key << "time_boot_ms" << QString::number(package_scaled_pressure.time_boot_ms);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_ATTITUDE:                                    //30
		mavlink_attitude_t package_attitude;
		memcpy(&package_attitude, data, MAVLINK_MSG_ID_ATTITUDE_LEN);
		key << "pitch" << QString::number(package_attitude.pitch);
		detail.append(key);  key.clear();
		key << "pitchspeed" << QString::number(package_attitude.pitchspeed);
		detail.append(key);  key.clear();
		key << "roll" << QString::number(package_attitude.roll);
		detail.append(key);  key.clear();
		key << "rollspeed" << QString::number(package_attitude.rollspeed);
		detail.append(key);  key.clear();
		key << "yaw" << QString::number(package_attitude.yaw);
		detail.append(key);  key.clear();
		key << "yawspeed" << QString::number(package_attitude.yawspeed);
		detail.append(key);  key.clear();
		key << "time_boot_ms" << QString::number(package_attitude.time_boot_ms);
		detail.append(key);  key.clear();
		boot_time_ms = package_attitude.time_boot_ms;
		return detail;
		break;

	case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:                         //31

		break;

	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:                         //33
		mavlink_global_position_int_t package_global_position_int;
		memcpy(&package_global_position_int, data, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
		key << "alt" << QString::number(package_global_position_int.alt,10);
		detail.append(key);  key.clear();
		key << "hdg" << QString::number(package_global_position_int.hdg,10);
		detail.append(key);  key.clear();
		key << "lat" << QString::number(package_global_position_int.lat,10);
		detail.append(key);  key.clear();
		key << "lon" << QString::number(package_global_position_int.lon,10);
		detail.append(key);  key.clear();
		key << "relative_alt" << QString::number(package_global_position_int.relative_alt,10);
		detail.append(key);  key.clear();
		key << "time_boot_ms" << QString::number(package_global_position_int.time_boot_ms,10);
		detail.append(key);  key.clear();
		key << "vx" << QString::number(package_global_position_int.vx,10);
		detail.append(key);  key.clear();
		key << "vy" << QString::number(package_global_position_int.vy,10);
		detail.append(key);  key.clear();
		key << "vz" << QString::number(package_global_position_int.vz);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_RC_CHANNELS_RAW:                             //35
		mavlink_rc_channels_raw_t package_rc_channels_raw;
		memcpy(&package_rc_channels_raw, data, MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN);
		key << "chan1_raw" << QString::number(package_rc_channels_raw.chan1_raw);
		detail.append(key);  key.clear();
		key << "chan2_raw" << QString::number(package_rc_channels_raw.chan2_raw);
		detail.append(key);  key.clear();
		key << "chan3_raw" << QString::number(package_rc_channels_raw.chan3_raw);
		detail.append(key);  key.clear();
		key << "chan4_raw" << QString::number(package_rc_channels_raw.chan4_raw);
		detail.append(key);  key.clear();
		key << "chan5_raw" << QString::number(package_rc_channels_raw.chan5_raw);
		detail.append(key);  key.clear();
		key << "chan6_raw" << QString::number(package_rc_channels_raw.chan6_raw);
		detail.append(key);  key.clear();
		key << "chan7_raw" << QString::number(package_rc_channels_raw.chan7_raw);
		detail.append(key);  key.clear();
		key << "chan8_raw" << QString::number(package_rc_channels_raw.chan8_raw);
		detail.append(key);  key.clear();
		key << "port" << QString::number(package_rc_channels_raw.port);
		detail.append(key);  key.clear();
		key << "rssi" << QString::number(package_rc_channels_raw.rssi);
		detail.append(key);  key.clear();
		key << "time_boot_ms" << QString::number(package_rc_channels_raw.time_boot_ms);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:                            //36
		mavlink_servo_output_raw_t package_servo_output_raw;
		memcpy(&package_servo_output_raw, data, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN);
		key << "port" << QString::number(package_servo_output_raw.port);
		detail.append(key);  key.clear();
		key << "servo1_raw" << QString::number(package_servo_output_raw.servo1_raw);
		detail.append(key);  key.clear();
		key << "servo2_raw" << QString::number(package_servo_output_raw.servo2_raw);
		detail.append(key);  key.clear();
		key << "servo3_raw" << QString::number(package_servo_output_raw.servo3_raw);
		detail.append(key);  key.clear();
		key << "servo4_raw" << QString::number(package_servo_output_raw.servo4_raw);
		detail.append(key);  key.clear();
		key << "servo5_raw" << QString::number(package_servo_output_raw.servo5_raw);
		detail.append(key);  key.clear();
		key << "servo6_raw" << QString::number(package_servo_output_raw.servo6_raw);
		detail.append(key);  key.clear();
		key << "servo7_raw" << QString::number(package_servo_output_raw.servo7_raw);
		detail.append(key);  key.clear();
		key << "servo8_raw" << QString::number(package_servo_output_raw.servo8_raw);
		detail.append(key);  key.clear();
		key << "servo9_raw" << QString::number(package_servo_output_raw.servo9_raw);
		detail.append(key);  key.clear();
		key << "servo10_raw" << QString::number(package_servo_output_raw.servo10_raw);
		detail.append(key);  key.clear();
		key << "servo11_raw" << QString::number(package_servo_output_raw.servo11_raw);
		detail.append(key);  key.clear();
		key << "servo12_raw" << QString::number(package_servo_output_raw.servo12_raw);
		detail.append(key);  key.clear();
		key << "servo13_raw" << QString::number(package_servo_output_raw.servo13_raw);
		detail.append(key);  key.clear();
		key << "servo14_raw" << QString::number(package_servo_output_raw.servo14_raw);
		detail.append(key);  key.clear();
		key << "servo15_raw" << QString::number(package_servo_output_raw.servo15_raw);
		detail.append(key);  key.clear();
		key << "servo16_raw" << QString::number(package_servo_output_raw.servo16_raw);
		detail.append(key);  key.clear();
		key << "time_usec" << QString::number(package_servo_output_raw.time_usec);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM:                                //39
		mavlink_mission_item_t package_mission_item;
		memcpy(&package_mission_item, data, MAVLINK_MSG_ID_MISSION_ITEM_LEN);
		key << "autocontinue" << QString::number(package_mission_item.autocontinue);
		detail.append(key);  key.clear();
		key << "command" << QString::number(package_mission_item.command);
		detail.append(key);  key.clear();
		key << "current" << QString::number(package_mission_item.current);
		detail.append(key);  key.clear();
		key << "frame" << QString::number(package_mission_item.frame);
		detail.append(key);  key.clear();
		key << "param1" << QString::number(package_mission_item.param1);
		detail.append(key);  key.clear();
		key << "param2" << QString::number(package_mission_item.param2);
		detail.append(key);  key.clear();
		key << "param3" << QString::number(package_mission_item.param3);
		detail.append(key);  key.clear();
		key << "param4" << QString::number(package_mission_item.param4);
		detail.append(key);  key.clear();
		key << "seq" << QString::number(package_mission_item.seq);
		detail.append(key);  key.clear();
		key << "target_component" << QString::number(package_mission_item.target_component);
		detail.append(key);  key.clear();
		key << "target_system" << QString::number(package_mission_item.target_system);
		detail.append(key);  key.clear();
		key << "x" << QString::number(package_mission_item.x);
		detail.append(key);  key.clear();
		key << "y" << QString::number(package_mission_item.y);
		detail.append(key);  key.clear();
		key << "z" << QString::number(package_mission_item.z);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_MISSION_CURRENT:                             //42
		mavlink_mission_current_t package_mission_current;
		memcpy(&package_mission_current, data, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
		key << "seq" << QString::number(package_mission_current.seq,10);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_MISSION_COUNT:                               //44
		mavlink_mission_count_t package_mission_count;
		memcpy(&package_mission_count, data, MAVLINK_MSG_ID_MISSION_COUNT_LEN);
		key << "count" << QString::number(package_mission_count.count);
		detail.append(key);  key.clear();
		key << "target_component" << QString::number(package_mission_count.target_component);
		detail.append(key);  key.clear();
		key << "target_system" << QString::number(package_mission_count.target_system);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:                       //62
		mavlink_nav_controller_output_t package_nav_controller_output;
		memcpy(&package_nav_controller_output, data, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);
		key << "alt_error" << QString::number(package_nav_controller_output.alt_error);
		detail.append(key);  key.clear();
		key << "aspd_error" << QString::number(package_nav_controller_output.aspd_error);
		detail.append(key);  key.clear();
		key << "nav_bearing" << QString::number(package_nav_controller_output.nav_bearing);
		detail.append(key);  key.clear();
		key << "nav_pitch" << QString::number(package_nav_controller_output.nav_pitch);
		detail.append(key);  key.clear();
		key << "nav_roll" << QString::number(package_nav_controller_output.nav_roll);
		detail.append(key);  key.clear();
		key << "target_bearing" << QString::number(package_nav_controller_output.target_bearing);
		detail.append(key);  key.clear();
		key << "wp_dist" << QString::number(package_nav_controller_output.wp_dist);
		detail.append(key);  key.clear();
		key << "xtrack_error" << QString::number(package_nav_controller_output.xtrack_error);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_RC_CHANNELS:                                 //65
		mavlink_rc_channels_t package_rc_channels;
		memcpy(&package_rc_channels, data, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
		key << "chan1_raw" << QString::number(package_rc_channels.chan1_raw);
		detail.append(key);  key.clear();
		key << "chan2_raw" << QString::number(package_rc_channels.chan2_raw);
		detail.append(key);  key.clear();
		key << "chan3_raw" << QString::number(package_rc_channels.chan3_raw);
		detail.append(key);  key.clear();
		key << "chan4_raw" << QString::number(package_rc_channels.chan4_raw);
		detail.append(key);  key.clear();
		key << "chan5_raw" << QString::number(package_rc_channels.chan5_raw);
		detail.append(key);  key.clear();
		key << "chan6_raw" << QString::number(package_rc_channels.chan6_raw);
		detail.append(key);  key.clear();
		key << "chan7_raw" << QString::number(package_rc_channels.chan7_raw);
		detail.append(key);  key.clear();
		key << "chan8_raw" << QString::number(package_rc_channels.chan8_raw);
		detail.append(key);  key.clear();
		key << "chan9_raw" << QString::number(package_rc_channels.chan9_raw);
		detail.append(key);  key.clear();
		key << "chan10_raw" << QString::number(package_rc_channels.chan10_raw);
		detail.append(key);  key.clear();
		key << "chan11_raw" << QString::number(package_rc_channels.chan11_raw);
		detail.append(key);  key.clear();
		key << "chan12_raw" << QString::number(package_rc_channels.chan12_raw);
		detail.append(key);  key.clear();
		key << "chan13_raw" << QString::number(package_rc_channels.chan13_raw);
		detail.append(key);  key.clear();
		key << "chan14_raw" << QString::number(package_rc_channels.chan14_raw);
		detail.append(key);  key.clear();
		key << "chan15_raw" << QString::number(package_rc_channels.chan15_raw);
		detail.append(key);  key.clear();
		key << "chan16_raw" << QString::number(package_rc_channels.chan16_raw);
		detail.append(key);  key.clear();
		key << "chan17_raw" << QString::number(package_rc_channels.chan17_raw);
		detail.append(key);  key.clear();
		key << "chan18_raw" << QString::number(package_rc_channels.chan18_raw);
		detail.append(key);  key.clear();
		key << "chancount" << QString::number(package_rc_channels.chancount);
		detail.append(key);  key.clear();
		key << "rssi" << QString::number(package_rc_channels.rssi);
		detail.append(key);  key.clear();
		key << "time_boot_ms" << QString::number(package_rc_channels.time_boot_ms);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_VFR_HUD:                                     //74
		mavlink_vfr_hud_t package_vfr_hud;
		memcpy(&package_vfr_hud, data, MAVLINK_MSG_ID_VFR_HUD_LEN);
		key << "airspeed" << QString::number(package_vfr_hud.airspeed);
		detail.append(key);  key.clear();
		key << "alt" << QString::number(package_vfr_hud.alt);
		detail.append(key);  key.clear();
		key << "climb" << QString::number(package_vfr_hud.climb);
		detail.append(key);  key.clear();
		key << "groundspeed" << QString::number(package_vfr_hud.groundspeed);
		detail.append(key);  key.clear();
		key << "heading" << QString::number(package_vfr_hud.heading);
		detail.append(key);  key.clear();
		key << "throttle" << QString::number(package_vfr_hud.throttle);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_COMMAND_ACK:                                 //77
		mavlink_command_ack_t package_command_ack;
		memcpy(&package_command_ack, data, MAVLINK_MSG_ID_COMMAND_ACK_LEN);
		key << "command" << QString::number(package_command_ack.command);
		detail.append(key);  key.clear();
		key << "result" << QString::number(package_command_ack.result);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_SCALED_IMU2:                                //116
		mavlink_scaled_imu2_t package_scaled_imu2;
		memcpy(&package_scaled_imu2, data, MAVLINK_MSG_ID_SCALED_IMU2_LEN);
		key << "time_boot_ms" << QString::number(package_scaled_imu2.time_boot_ms);
		detail.append(key);  key.clear();
		key << "xacc" << QString::number(package_scaled_imu2.xacc);
		detail.append(key);  key.clear();
		key << "xgyro" << QString::number(package_scaled_imu2.xgyro);
		detail.append(key);  key.clear();
		key << "xmag" << QString::number(package_scaled_imu2.xmag);
		detail.append(key);  key.clear();
		key << "yacc" << QString::number(package_scaled_imu2.yacc);
		detail.append(key);  key.clear();
		key << "ygyro" << QString::number(package_scaled_imu2.ygyro);
		detail.append(key);  key.clear();
		key << "ymag" << QString::number(package_scaled_imu2.ymag);
		detail.append(key);  key.clear();
		key << "zacc" << QString::number(package_scaled_imu2.zacc);
		detail.append(key);  key.clear();
		key << "zgyro" << QString::number(package_scaled_imu2.zgyro);
		detail.append(key);  key.clear();
		key << "zmag" << QString::number(package_scaled_imu2.zmag);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_POWER_STATUS:                               //125
		mavlink_power_status_t package_power_status;
		memcpy(&package_power_status, data, MAVLINK_MSG_ID_POWER_STATUS_LEN);
		key << "flags" << QString::number(package_power_status.flags);
		detail.append(key);  key.clear();
		key << "Vcc" << QString::number(package_power_status.Vcc);
		detail.append(key);  key.clear();
		key << "Vservo" << QString::number(package_power_status.Vservo);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_AUTOPILOT_VERSION:                          //148
		mavlink_autopilot_version_t package_autopilot_version;
		memcpy(&package_autopilot_version, data, MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN);
		key << "board_version" << QString::number(package_autopilot_version.board_version);
		detail.append(key);  key.clear();
		key << "capabilities" << QString::number(package_autopilot_version.capabilities);
		detail.append(key);  key.clear();
		key << "flight_custom_version" << QString((char *)package_autopilot_version.flight_custom_version);
		detail.append(key);  key.clear();
		key << "flight_sw_version" << QString::number(package_autopilot_version.flight_sw_version);
		detail.append(key);  key.clear();
		key << "middleware_custom_version" << QString((char *)package_autopilot_version.middleware_custom_version);
		detail.append(key);  key.clear();
		key << "middleware_sw_version" << QString::number(package_autopilot_version.middleware_sw_version);
		detail.append(key);  key.clear();
		key << "os_custom_version" << QString((char *)package_autopilot_version.os_custom_version);
		detail.append(key);  key.clear();
		key << "os_sw_version" << QString::number(package_autopilot_version.os_sw_version);
		detail.append(key);  key.clear();
		key << "product_id" << QString::number(package_autopilot_version.product_id);
		detail.append(key);  key.clear();
		key << "uid" << QString::number(package_autopilot_version.uid);
		detail.append(key);  key.clear();
		key << "vendor_id" << QString::number(package_autopilot_version.vendor_id);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_SENSOR_OFFSETS:                             //150
		mavlink_sensor_offsets_t package_sensor_offsets;
		memcpy(&package_sensor_offsets, data, MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN);
		key << "accel_cal_x" << QString::number(package_sensor_offsets.accel_cal_x);
		detail.append(key);  key.clear();
		key << "accel_cal_y" << QString::number(package_sensor_offsets.accel_cal_y);
		detail.append(key);  key.clear();
		key << "accel_cal_z" << QString::number(package_sensor_offsets.accel_cal_z);
		detail.append(key);  key.clear();
		key << "gyro_cal_x" << QString::number(package_sensor_offsets.gyro_cal_x);
		detail.append(key);  key.clear();
		key << "gyro_cal_y" << QString::number(package_sensor_offsets.gyro_cal_y);
		detail.append(key);  key.clear();
		key << "gyro_cal_z" << QString::number(package_sensor_offsets.gyro_cal_z);
		detail.append(key);  key.clear();
		key << "mag_declination" << QString::number(package_sensor_offsets.mag_declination);
		detail.append(key);  key.clear();
		key << "mag_ofs_x" << QString::number(package_sensor_offsets.mag_ofs_x);
		detail.append(key);  key.clear();
		key << "mag_ofs_y" << QString::number(package_sensor_offsets.mag_ofs_y);
		detail.append(key);  key.clear();
		key << "mag_ofs_z" << QString::number(package_sensor_offsets.mag_ofs_z);
		detail.append(key);  key.clear();
		key << "raw_press" << QString::number(package_sensor_offsets.raw_press);
		detail.append(key);  key.clear();
		key << "raw_temp" << QString::number(package_sensor_offsets.raw_temp);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_MEMINFO:                                    //152
		mavlink_meminfo_t package_meminfo;
		memcpy(&package_meminfo, data, MAVLINK_MSG_ID_MEMINFO_LEN);
		key << "brkval" << QString::number(package_meminfo.brkval);
		detail.append(key);  key.clear();
		key << "freemem" << QString::number(package_meminfo.freemem);
		detail.append(key);  key.clear();
		key << "freemem32" << QString::number(package_meminfo.freemem32);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_AHRS:                                        //163
		mavlink_ahrs_t package_ahrs;
		memcpy(&package_ahrs, data, MAVLINK_MSG_ID_AHRS_LEN);
		key << "accel_weight" << QString::number(package_ahrs.accel_weight);
		detail.append(key);  key.clear();
		key << "error_rp" << QString::number(package_ahrs.error_rp);
		detail.append(key);  key.clear();
		key << "error_yaw" << QString::number(package_ahrs.error_yaw);
		detail.append(key);  key.clear();
		key << "omegaIx" << QString::number(package_ahrs.omegaIx);
		detail.append(key);  key.clear();
		key << "omegaIy" << QString::number(package_ahrs.omegaIy);
		detail.append(key);  key.clear();
		key << "omegaIz" << QString::number(package_ahrs.omegaIz);
		detail.append(key);  key.clear();
		key << "renorm_val" << QString::number(package_ahrs.renorm_val);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_HWSTATUS:                                    //165
		mavlink_hwstatus_t package_hwstatus;
		memcpy(&package_hwstatus, data, MAVLINK_MSG_ID_HWSTATUS_LEN);
		key << "I2Cerr" << QString::number(package_hwstatus.I2Cerr);
		detail.append(key);  key.clear();
		key << "Vcc" << QString::number(package_hwstatus.Vcc);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_AHRS2:                                       //178
		mavlink_ahrs2_t package_ahrs2;
		memcpy(&package_ahrs2, data, MAVLINK_MSG_ID_AHRS2_LEN);
		key << "altitude" << QString::number(package_ahrs2.altitude);
		detail.append(key);  key.clear();
		key << "lat" << QString::number(package_ahrs2.lat);
		detail.append(key);  key.clear();
		key << "lng" << QString::number(package_ahrs2.lng);
		detail.append(key);  key.clear();
		key << "pitch" << QString::number(package_ahrs2.pitch);
		detail.append(key);  key.clear();
		key << "roll" << QString::number(package_ahrs2.roll);
		detail.append(key);  key.clear();
		key << "yaw" << QString::number(package_ahrs2.yaw);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_AHRS3:                                       //182
		mavlink_ahrs3_t package_ahrs3;
		memcpy(&package_ahrs3, data, MAVLINK_MSG_ID_AHRS3_LEN);
		key << "altitude" << QString::number(package_ahrs3.altitude);
		detail.append(key);  key.clear();
		key << "lat" << QString::number(package_ahrs3.lat);
		detail.append(key);  key.clear();
		key << "lng" << QString::number(package_ahrs3.lng);
		detail.append(key);  key.clear();
		key << "pitch" << QString::number(package_ahrs3.pitch);
		detail.append(key);  key.clear();
		key << "roll" << QString::number(package_ahrs3.roll);
		detail.append(key);  key.clear();
		key << "yaw" << QString::number(package_ahrs3.yaw);
		detail.append(key);  key.clear();
		key << "v1" << QString::number(package_ahrs3.v1);
		detail.append(key);  key.clear();
		key << "v2" << QString::number(package_ahrs3.v2);
		detail.append(key);  key.clear();
		key << "v3" << QString::number(package_ahrs3.v3);
		detail.append(key);  key.clear();
		key << "v4" << QString::number(package_ahrs3.v4);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_EKF_STATUS_REPORT:                           //193
		mavlink_ekf_status_report_t package_ekf_status_report;
		memcpy(&package_ekf_status_report, data, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN);
		key << "compass_variance" << QString::number(package_ekf_status_report.compass_variance);
		detail.append(key);  key.clear();
		key << "flags" << QString::number(package_ekf_status_report.flags);
		detail.append(key);  key.clear();
		key << "pos_horiz_variance" << QString::number(package_ekf_status_report.pos_horiz_variance);
		detail.append(key);  key.clear();
		key << "pos_vert_variance" << QString::number(package_ekf_status_report.pos_vert_variance);
		detail.append(key);  key.clear();
		key << "terrain_alt_variance" << QString::number(package_ekf_status_report.terrain_alt_variance);
		detail.append(key);  key.clear();
		key << "velocity_variance" << QString::number(package_ekf_status_report.velocity_variance);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_VIBRATION:                                  //241
		mavlink_vibration_t package_vibration;
		memcpy(&package_vibration, data, MAVLINK_MSG_ID_VIBRATION_LEN);
		key << "clipping_0" << QString::number(package_vibration.clipping_0);
		detail.append(key);  key.clear();
		key << "clipping_1" << QString::number(package_vibration.clipping_1);
		detail.append(key);  key.clear();
		key << "clipping_2" << QString::number(package_vibration.clipping_2);
		detail.append(key);  key.clear();
		key << "time_usec" << QString::number(package_vibration.time_usec);
		detail.append(key);  key.clear();
		key << "vibration_x" << QString::number(package_vibration.vibration_x);
		detail.append(key);  key.clear();
		key << "vibration_y" << QString::number(package_vibration.vibration_y);
		detail.append(key);  key.clear();
		key << "vibration_z" << QString::number(package_vibration.vibration_z);
		detail.append(key);  key.clear();
		return detail;
		break;

	case MAVLINK_MSG_ID_STATUSTEXT:                                  //253
		mavlink_statustext_t package_statustext;
		memcpy(&package_statustext, data, MAVLINK_MSG_ID_STATUSTEXT_LEN);
		key << "severity" << QString::number(package_statustext.severity);
		detail.append(key);  key.clear();
		key << "text" << QString(package_statustext.text);
		detail.append(key);  key.clear();
		return detail;
		break;
	default:
		break;
	}


	

   return detail;

}


float MavlinkInspector::GetFrequency(uint8_t msgid) {
	float frequency;
	if (Interval[msgid] == 0)
	{
		frequency = 0;
		Interval[msgid] = time.currentTime().hour() * 3600 + time.currentTime().minute() * 60 + time.currentTime().second() + 1.0*time.currentTime().msec() / 1000;
	}
	else
	{
		frequency = time.currentTime().hour() * 3600 + time.currentTime().minute() * 60 + time.currentTime().second() + 1.0*time.currentTime().msec() / 1000 - Interval[msgid];
		Interval[msgid] += frequency;
		frequency = 1/frequency;
	}
	return frequency;
}


uint32_t MavlinkInspector::Get_sys_time() {
	return boot_time_ms;
}