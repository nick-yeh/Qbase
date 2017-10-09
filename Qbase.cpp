#include "Qbase.h"
#include <stdint.h>
#include <QDir>
#include <QtWebEngineWidgets/QWebEngineView>
#include "DroneInfo.h"

//#include <QUrl>
//#include <QVariant>
//#include <QAbstractEventDispatcher>


Qbase::Qbase(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	UpdateSerialInfo();
	connect(ui.Inspector_btn, SIGNAL(clicked()), this, SLOT(on_openMavlinkInspector_Btn_clicked()));
	connect(ui.Cmd_btn, SIGNAL(clicked()), this, SLOT(on_set_mode_btn_clicked()));
	connect(ui.Connect_btn, SIGNAL(clicked()), this, SLOT(on_openPortBtn_clicked()));
	connect(ui.Test_btn, SIGNAL(clicked()), this, SLOT(on_openTestBtn_clicked()));
	connect(ui.Arm_btn, SIGNAL(clicked()), this, SLOT(on_openArmBtn_clicked()));
	connect(p_timer, SIGNAL(timeout()), this, SLOT(read_Com()));
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(send_guide_attitude()));
	connect(&m_refresh_timer, SIGNAL(timeout()), this, SLOT(on_update()));
	connect(ui.clearMarker_btn, SIGNAL(clicked()), this, SLOT(clearAllMarker()));
	connect(ui.setMarker_btn, SIGNAL(clicked()), this, SLOT(on_setMarkerOnMapBtn_clicked()));

	//yhk added
	connect(ui.FlyToHere_btn, SIGNAL(clicked()), this, SLOT(on_flyToHereBtn_clicked()));
	connect(ui.Takeoff_btn, SIGNAL(clicked()), this, SLOT(on_takeOffBtn_clicked()));
	connect(ui.Land_btn, SIGNAL(clicked()), this, SLOT(on_landBtn_clicked()));
	connect(ui.Disarm_btn, SIGNAL(clicked()), this, SLOT(on_disarmBtn_clicked())); 
	connect(ui.FlyNorth_btn, SIGNAL(clicked()), this, SLOT(on_flyNorthBtn_clicked()));
	connect(ui.FlySouth_btn, SIGNAL(clicked()), this, SLOT(on_flySouthBtn_clicked()));
	connect(ui.FlyWest_btn, SIGNAL(clicked()), this, SLOT(on_flyWestBtn_clicked()));
	connect(ui.FlyEast_btn, SIGNAL(clicked()), this, SLOT(on_flyEastBtn_clicked()));
	connect(ui.FlyUp_btn, SIGNAL(clicked()), this, SLOT(on_flyUpBtn_clicked()));
	connect(ui.FlyDown_btn, SIGNAL(clicked()), this, SLOT(on_flyDownBtn_clicked()));
	//

	ui.Baudrate_combo->setCurrentIndex(1);
	m_mp.setInspector(&m_mi);
	m_refresh_timer.start(1000);
	
	count = 0;

	view = new QWebEngineView(ui.widget);
	QString mapHtml = QDir::currentPath() + "/MapFile/new 1.html";
	view->page()->load(mapHtml);
	connect(view->page(), SIGNAL(loadFinished(bool)), this, SLOT(mapLoadFinished()));
}


void Qbase::UpdateSerialInfo()
{
	ui.Portname_combo->clear();
	foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
		QString s = QObject::tr("Port: ") + info.portName();
		//if (info.description() == "PX4 FMU" && !info.isBusy())
		if(true)
		{
			ui.Portname_combo->addItem(info.portName());
		}
	}
}

void Qbase::UpdateDroneInfo()
{
    DroneInfo info = m_mi.Get_drone_info(ui.Choose_id_combo->currentText());
	QString info_txt = "Latitude: " +  QString::number(info.latitude) + "\r\n" + "Longitude: " + QString::number(info.longitude) + "\r\n" + "Timestamp: " + QString::number((long)info.timestamp);
	if (ui.Choose_id_combo->currentText() != "")
	{
		ui.Info_label->setText(info_txt);
	}
	else
	{
		ui.Info_label->setText("");
	}

}

void Qbase::UpdateMapMarker()
{
	if (ui.Choose_id_combo->currentText() != "")
	{
		//clear all markers
		QString method = QString("myClearAllMarker()");
		view->page()->runJavaScript(method);
		
		//add
		double lng = ui.LngEdit->text().toDouble();
		double lat = ui.LatEdit->text().toDouble();
		if (lng != 0 && lat != 0)
		{
			QString method = QString("myAddMarker(\"%1\", \"%2\")").arg(QString::number((double)lng, 'g', 12)).arg(QString::number((double)lat, 'g', 12));
			view->page()->runJavaScript(method);


		}
		setOneMarker();
	}
	else
	{
		
	}
}

void Qbase::UpdateDroneID()
{
	QString current_choice;
	QVector<QString> drone_id = m_mi.Get_id_list();

	current_choice = ui.Choose_id_combo->currentText();
	if (current_choice != "")
	{
		ui.Choose_id_combo->clear();
		for (int i = 0; i < drone_id.length(); i++)
		{
			ui.Choose_id_combo->addItem(drone_id[i]);
			if (drone_id[i] == current_choice)
			{
				ui.Choose_id_combo->setCurrentText(drone_id[i]);
			}
		}
	}
	else
	{
		ui.Choose_id_combo->clear();
		for (int i = 0; i < drone_id.length(); i++)
		{
			ui.Choose_id_combo->addItem(drone_id[i]);
		}
	}

	
}

void Qbase::on_openPortBtn_clicked()
{
	if (!m_serial.isOpen())
	{	
		int tmp_baudrate;
		m_serial.setPortName(ui.Portname_combo->currentText());	//设置COM口
		switch (ui.Baudrate_combo->currentIndex())
		{
		case 0:tmp_baudrate = QSerialPort::Baud115200; break;
		case 1:tmp_baudrate = QSerialPort::Baud57600; break;
		case 2:tmp_baudrate = QSerialPort::Baud38400; break;
		case 3:tmp_baudrate = QSerialPort::Baud19200; break;
		case 4:tmp_baudrate = QSerialPort::Baud9600; break;
		default:
			break;
		}
		m_serial.setBaudRate(tmp_baudrate, QSerialPort::AllDirections);//设置波特率和读写方向
		m_serial.setDataBits(QSerialPort::Data8);		//数据位为8位
		m_serial.setFlowControl(QSerialPort::NoFlowControl);//无流控制
		m_serial.setParity(QSerialPort::NoParity);	//无校验位
		m_serial.setStopBits(QSerialPort::OneStop);	//一位停止位
		m_serial.close();					//先关串口，再打开，可以保证串口不被其它函数占用。
		if (m_serial.open(QIODevice::ReadWrite))		//以可读写的方式打开串口
		{
			connect(&m_serial, SIGNAL(readyRead()), this, SLOT(read_Com()));	//把串口的readyRead()信号绑定到read_Com()这个槽函数上
			ui.Portname_combo->setDisabled(true);   //按下“Connect”后，禁用串口选择
			ui.Baudrate_combo->setDisabled(true);   //按下“Connect”后，禁用波特率选择
			ui.Connect_btn->setText("Close");		//按下“Connect”后，按键显示为“Close”
			p_timer->start(5);
		}
	}
	else {
		p_timer->stop();
		m_serial.close();					//关串口
		ui.Connect_btn->setText("Connect");		//按下“Close”后，按键显示为“Connect”
		ui.Portname_combo->setEnabled(true);	//按下“Close”后，COM口可被修改
		ui.Baudrate_combo->setEnabled(true);   //按下“Connect”后，波特率可以修改
		UpdateSerialInfo();
	}
}

void Qbase::read_Com()
{
	QByteArray header = m_serial.read(1);//在缓冲区中读一个byte
	if (header.length()<=0) return;
	if ((uchar)header.at(0) == 0xfe)
	{
		QByteArray data;
		QByteArray length = m_serial.read(MAVLINK_MESSAGE_PAYLOAD_LENGTH_BYTE);
		if (length.length() <= 0) return;
		QByteArray info = m_serial.read(4);
		QByteArray payload = m_serial.read(length.at(0));
		QByteArray CK = m_serial.read(2);
		data.append(header);
		data.append(length);
		data.append(info);
		data.append(payload);
		uint16_t crc = ((CK.data()[1]&0xFF) << 8) | (CK.data()[0] & 0xFF);
		if (crc != CRC_gengerate((unsigned char*)data.data())) return;
		m_mp.Parse((uint8_t*)data.data());
	}
}


//---------------------------------------------
void Qbase::mapLoadFinished()
{
	if (ui.Choose_id_combo->currentText() != "")
	{
		DroneInfo info = m_mi.Get_drone_info(ui.Choose_id_combo->currentText());
		double lng = info.longitude;
		double lat = info.latitude;

		QString method = QString("myAddMarker(\"%1\", \"%2\")").arg(QString::number((double)lng,'g',12)).arg(QString::number((double)lat, 'g', 12));
		view->page()->runJavaScript(method);
	}
}

void Qbase::clearAllMarker()
{
	QString method = QString("myClearAllMarker()");
	view->page()->runJavaScript(method);
}

void Qbase::setOneMarker()
{
	DroneInfo info = m_mi.Get_drone_info(ui.Choose_id_combo->currentText());
	double lng = info.longitude;
	double lat = info.latitude;
	QString method = QString("myAddMarker(\"%1\", \"%2\")").arg(QString::number((double)lng, 'g', 12)).arg(QString::number((double)lat, 'g', 12));
	view->page()->runJavaScript(method);
}
//------------------------------------------

void Qbase::on_setMarkerOnMapBtn_clicked()
{
	double lng = ui.LngEdit->text().toDouble();
	double lat = ui.LatEdit->text().toDouble();
	QString method = QString("myAddMarker(\"%1\", \"%2\")").arg(QString::number((double)lng, 'g', 12)).arg(QString::number((double)lat, 'g', 12));
	view->page()->runJavaScript(method);
}


void Qbase::on_openMavlinkInspector_Btn_clicked()
{
		m_mi.show();
		m_mi.activateWindow();
}


void Qbase::on_set_mode_btn_clicked()
{ 
	char buf[MAVLINK_MSG_PACKAGE_SET_MODE_LEN];

	if (ui.Flight_mode_combo->currentText() == "STABILIZE")
	{
		m_md.Make_Command_Set_Mode(0,81,m_mp.Get_sys_id(),buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "GUIDED_NOGPS")
	{
		m_md.Make_Command_Set_Mode(20,81, m_mp.Get_sys_id(), buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "LOITER")
	{
		m_md.Make_Command_Set_Mode(5,89, m_mp.Get_sys_id(), buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "AUTO")
	{
		m_md.Make_Command_Set_Mode(3,89, m_mp.Get_sys_id(), buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "RTL")
	{
		m_md.Make_Command_Set_Mode(6, 89, m_mp.Get_sys_id(), buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "ATL_HOLD")
	{
		m_md.Make_Command_Set_Mode(2, 81, m_mp.Get_sys_id(), buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "LAND")
	{
		m_md.Make_Command_Set_Mode(9, 81, m_mp.Get_sys_id(), buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "GUIDED")
	{
		m_md.Make_Command_Set_Mode(4, 89, m_mp.Get_sys_id(), buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "CIRCLE")
	{
		m_md.Make_Command_Set_Mode(7, 89, m_mp.Get_sys_id(), buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "SPORT")
	{
		m_md.Make_Command_Set_Mode(13, 81, m_mp.Get_sys_id(), buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "DRIFT")
	{
		m_md.Make_Command_Set_Mode(11, 81, m_mp.Get_sys_id(), buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "FLIP")
	{
		m_md.Make_Command_Set_Mode(14, 81, m_mp.Get_sys_id(), buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "POSHOLD")
	{
		m_md.Make_Command_Set_Mode(16, 89, m_mp.Get_sys_id(), buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "BRAKE")
	{
		m_md.Make_Command_Set_Mode(17, 81, m_mp.Get_sys_id(), buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "THROW")
	{
		m_md.Make_Command_Set_Mode(18, 81, m_mp.Get_sys_id(), buf);
	}
	else if (ui.Flight_mode_combo->currentText() == "AVOID_ADSB")
	{
		m_md.Make_Command_Set_Mode(19, 81, m_mp.Get_sys_id(), buf);
	}

	m_serial.write(buf, MAVLINK_MSG_PACKAGE_SET_MODE_LEN);
}



void Qbase::on_openTestBtn_clicked() {

	/*
	if (m_timer.isActive()) m_timer.stop();
	else m_timer.start(200);


	uint16_t checksum;
	float buf[7];
	uint8_t ck[2];

	for (int i = 0; i<7; i++)
	{
		buf[i] = 0;
	}
	buf[5] = 1.0f;
	char data[14];
	data[0] = 254;
	data[1] = 6;
	data[2] = 170;
	data[3] = 255;
	data[4] = 0;
	data[5] = 11;
	data[6] = 3;
	data[7] = 0;
	data[8] = 0;
	data[9] = 0;
	data[10] = 4;
	data[11] = 89;
	data[12] = 139;
	data[13] = 120;
	float temp = 1.0f;

	char data3[41];
	data3[0] = 254;
	data3[1] = 33;
	data3[2] = 200;
	data3[3] = 255;
	data3[4] = 0;
	data3[5] = 76;
	data3[6] = 0;
	data3[7] = 0;
	data3[8] = 0;
	data3[9] = 0;
	data3[10] = 0;
	data3[11] = 0;
	data3[12] = 0;
	data3[13] = 0;
	data3[14] = 0;
	data3[15] = 0;
	data3[16] = 0;
	data3[17] = 0;
	data3[18] = 0;
	data3[19] = 0;
	data3[20] = 0;
	data3[21] = 0;
	data3[22] = 0;
	data3[23] = 0;
	data3[24] = 128;
	data3[25] = 63;
	data3[26] = 0;
	data3[27] = 0;
	data3[28] = 0;
	data3[29] = 0;
	data3[30] = 0;
	data3[31] = 0;
	data3[32] = 0;
	data3[33] = 0;
	data3[34] = 241;
	data3[35] = 0;
	data3[36] = 4;
	data3[37] = 1;
	data3[38] = 0;
	data3[39] = 53;
	data3[40] = 32;


	//unsigned char* data2 = cmd.Make_Command_Long(4, 1, MAV_CMD_ATTITUDE_SETPOINT, 0 , 1000.0f,0,0,0,0,0,0);
	float quaternion[4];
	mavlink_euler_to_quaternion(0.0314, 0, 0, quaternion);
	char data2[MAVLINK_MSG_PACKAGE_SET_ATTITUDE_TARGET_LEN];
    m_md.Make_Command_Set_Attitude_Target(m_mi.Get_sys_time(), m_mp.Get_sys_id(), m_mp.Get_com_id(), 0x07, quaternion, 0, 0, 0, 0, data2);

	m_serial.write(data2, MAVLINK_MSG_PACKAGE_SET_ATTITUDE_TARGET_LEN);
	*/
}

void Qbase::on_flyToHereBtn_clicked()
{
	char buf[MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN];
	if (ui.LngEdit->text() != "" && ui.LatEdit->text() != "")
	{
		int lat = ui.LatEdit->text().toDouble()*10000000;
		int lng = ui.LngEdit->text().toDouble()*10000000;
		float alt = m_mi.Get_drone_info(ui.Choose_id_combo->currentText()).altitude;
		float vx = 0;
		float vy = 0;
		float vz = 0;
		float afx = 0;
		float afy = 0;
		float afz = 0;
		float yaw = 0;
		float yaw_rate = 0;
		uint16_t mask = TYPE_MASK_POSITION;
		uint8_t tar_sys = ui.Choose_id_combo->currentText().replace("vehicle", "").toInt();
		uint8_t tar_comp = m_mp.Get_com_id();
		uint8_t frame = TARGET_FRAME;

		m_md.Make_Command_Set_Position_Target_Global_Int(0, lat, lng, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate, mask, tar_sys, tar_comp, frame, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN);
	}
}

void Qbase::on_flyNorthBtn_clicked()
{
	char buf[MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN];
	if (ui.VelEdit->text() != "")
	{
		int lat = 0;
		int lng = 0;
		float alt = m_mi.Get_drone_info(ui.Choose_id_combo->currentText()).altitude;
		float vx = ui.VelEdit->text().toFloat();
		float vy = 0;
		float vz = 0;
		float afx = 0;
		float afy = 0;
		float afz = 0;
		float yaw = 0;
		float yaw_rate = 0;
		uint16_t mask = TYPE_MASK_VELOCITY;
		uint8_t tar_sys = ui.Choose_id_combo->currentText().replace("vehicle", "").toInt();
		uint8_t tar_comp = m_mp.Get_com_id();
		uint8_t frame = TARGET_FRAME;

		m_md.Make_Command_Set_Position_Target_Global_Int(0, lat, lng, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate, mask, tar_sys, tar_comp, frame, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN);
	}
}

void Qbase::on_flySouthBtn_clicked()
{
	char buf[MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN];
	if (ui.VelEdit->text() != "")
	{
		int lat = 0;
		int lng = 0;
		float alt = m_mi.Get_drone_info(ui.Choose_id_combo->currentText()).altitude;
		float vx = - ui.VelEdit->text().toFloat();
		float vy = 0;
		float vz = 0;
		float afx = 0;
		float afy = 0;
		float afz = 0;
		float yaw = 0;
		float yaw_rate = 0;
		uint16_t mask = TYPE_MASK_VELOCITY;
		uint8_t tar_sys = ui.Choose_id_combo->currentText().replace("vehicle", "").toInt();
		uint8_t tar_comp = m_mp.Get_com_id();
		uint8_t frame = TARGET_FRAME;

		m_md.Make_Command_Set_Position_Target_Global_Int(0, lat, lng, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate, mask, tar_sys, tar_comp, frame, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN);
	}
}

void Qbase::on_flyWestBtn_clicked()
{
	char buf[MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN];
	if (ui.VelEdit->text() != "")
	{
		int lat = 0;
		int lng = 0;
		float alt = m_mi.Get_drone_info(ui.Choose_id_combo->currentText()).altitude;
		float vx = 0;
		float vy = - ui.VelEdit->text().toFloat();
		float vz = 0;
		float afx = 0;
		float afy = 0;
		float afz = 0;
		float yaw = 0;
		float yaw_rate = 0;
		uint16_t mask = TYPE_MASK_VELOCITY;
		uint8_t tar_sys = ui.Choose_id_combo->currentText().replace("vehicle", "").toInt();
		uint8_t tar_comp = m_mp.Get_com_id();
		uint8_t frame = TARGET_FRAME;

		m_md.Make_Command_Set_Position_Target_Global_Int(0, lat, lng, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate, mask, tar_sys, tar_comp, frame, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN);
	}
}

void Qbase::on_flyEastBtn_clicked()
{
	char buf[MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN];
	if (ui.VelEdit->text() != "")
	{
		int lat = 0;
		int lng = 0;
		float alt = m_mi.Get_drone_info(ui.Choose_id_combo->currentText()).altitude;
		float vx = 0;
		float vy = ui.VelEdit->text().toFloat();
		float vz = 0;
		float afx = 0;
		float afy = 0;
		float afz = 0;
		float yaw = 0;
		float yaw_rate = 0;
		uint16_t mask = TYPE_MASK_VELOCITY;
		uint8_t tar_sys = ui.Choose_id_combo->currentText().replace("vehicle", "").toInt();
		uint8_t tar_comp = m_mp.Get_com_id();
		uint8_t frame = TARGET_FRAME;

		m_md.Make_Command_Set_Position_Target_Global_Int(0, lat, lng, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate, mask, tar_sys, tar_comp, frame, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN);
	}
}

void Qbase::on_flyUpBtn_clicked()
{
	char buf[MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN];
	if (ui.VelEdit->text() != "")
	{
		int lat = 0;
		int lng = 0;
		float alt = m_mi.Get_drone_info(ui.Choose_id_combo->currentText()).altitude;
		float vx = 0;
		float vy = 0;
		float vz = - ui.VelEdit->text().toFloat();
		float afx = 0;
		float afy = 0;
		float afz = 0;
		float yaw = 0;
		float yaw_rate = 0;
		uint16_t mask = TYPE_MASK_VELOCITY;
		uint8_t tar_sys = ui.Choose_id_combo->currentText().replace("vehicle", "").toInt();
		uint8_t tar_comp = m_mp.Get_com_id();
		uint8_t frame = TARGET_FRAME;

		m_md.Make_Command_Set_Position_Target_Global_Int(0, lat, lng, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate, mask, tar_sys, tar_comp, frame, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN);
	}
}

void Qbase::on_flyDownBtn_clicked()
{
	char buf[MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN];
	if (ui.VelEdit->text() != "")
	{
		int lat = 0;
		int lng = 0;
		float alt = m_mi.Get_drone_info(ui.Choose_id_combo->currentText()).altitude;
		float vx = 0;
		float vy = 0;
		float vz = ui.VelEdit->text().toFloat();
		float afx = 0;
		float afy = 0;
		float afz = 0;
		float yaw = 0;
		float yaw_rate = 0;
		uint16_t mask = TYPE_MASK_VELOCITY;
		uint8_t tar_sys = ui.Choose_id_combo->currentText().replace("vehicle", "").toInt();
		uint8_t tar_comp = m_mp.Get_com_id();
		uint8_t frame = TARGET_FRAME;

		m_md.Make_Command_Set_Position_Target_Global_Int(0, lat, lng, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate, mask, tar_sys, tar_comp, frame, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_SET_POSITION_TARGET_GLOBAL_INT_LEN);
	}
}


void Qbase::on_takeOffBtn_clicked()
{
	char buf[MAVLINK_MSG_PACKAGE_COMMAND_LONG_LEN];
	if (m_serial.isOpen())
	{
		int Hight = ui.HightEdit->text().toInt();
		m_md.Make_Command_Long(ui.Choose_id_combo->currentText().replace("vehicle", "").toInt(), m_mp.Get_com_id(), MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, Hight, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_COMMAND_LONG_LEN);
		m_md.Make_Command_Long(ui.Choose_id_combo->currentText().replace("vehicle", "").toInt(), m_mp.Get_com_id(), MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, Hight, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_COMMAND_LONG_LEN);
	}
}


void Qbase::on_landBtn_clicked()
{
	char buf[MAVLINK_MSG_PACKAGE_COMMAND_LONG_LEN];
	if (m_serial.isOpen())
	{
		m_md.Make_Command_Long(ui.Choose_id_combo->currentText().replace("vehicle", "").toInt(), m_mp.Get_com_id(), MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_COMMAND_LONG_LEN);
		m_md.Make_Command_Long(ui.Choose_id_combo->currentText().replace("vehicle", "").toInt(), m_mp.Get_com_id(), MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_COMMAND_LONG_LEN);
	}
}

void Qbase::on_openArmBtn_clicked()
{
	char buf[MAVLINK_MSG_PACKAGE_COMMAND_LONG_LEN];
	if (m_serial.isOpen())
	{
		m_md.Make_Command_Long(ui.Choose_id_combo->currentText().replace("vehicle", "").toInt(), m_mp.Get_com_id(), MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_COMMAND_LONG_LEN);
		m_md.Make_Command_Long(ui.Choose_id_combo->currentText().replace("vehicle", "").toInt(), m_mp.Get_com_id(), MAV_CMD_COMPONENT_ARM_DISARM, 1, 1, 0, 0, 0, 0, 0, 0, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_COMMAND_LONG_LEN);
	}
}

void Qbase::on_disarmBtn_clicked()
{
	char buf[MAVLINK_MSG_PACKAGE_COMMAND_LONG_LEN];
	if (m_serial.isOpen())
	{
		m_md.Make_Command_Long(ui.Choose_id_combo->currentText().replace("vehicle", "").toInt(), m_mp.Get_com_id(), MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_COMMAND_LONG_LEN);
		m_md.Make_Command_Long(ui.Choose_id_combo->currentText().replace("vehicle", "").toInt(), m_mp.Get_com_id(), MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0, 0, buf);
		m_serial.write(buf, MAVLINK_MSG_PACKAGE_COMMAND_LONG_LEN);
	}
}

void Qbase::send_guide_attitude(){
	float quaternion[4];
	count++;
	float angle = 0.157*(sin(1.0*count));
	mavlink_euler_to_quaternion(angle, 0, 0, quaternion);
	char data2[MAVLINK_MSG_PACKAGE_SET_ATTITUDE_TARGET_LEN];
	m_md.Make_Command_Set_Attitude_Target(ui.Choose_id_combo->currentText().replace("vehicle", "").toInt(), m_mp.Get_sys_id(), m_mp.Get_com_id(), 0x07, quaternion, 0, 0, 0, 30, data2);

	m_serial.write(data2, MAVLINK_MSG_PACKAGE_SET_ATTITUDE_TARGET_LEN);
}

void Qbase::on_update() {
	if (ui.Portname_combo->isEnabled())
	{
		UpdateSerialInfo();
	}

	UpdateDroneID();
	
	UpdateDroneInfo();

	UpdateMapMarker();

}


//使用AxWidget的方式：
/*-------------------------------------------
void Qbase::showBaiduMap()
{
	QString mapHtml = QDir::currentPath() + "/MapFile/new 1.html";
	webShow(mapHtml);
}

void Qbase::webShow(const QString &url)
{
	ui.axWidget->dynamicCall("Navigate(const QString&)", url);
}
*/