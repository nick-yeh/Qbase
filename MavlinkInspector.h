#pragma once

#include <QWidget>
#include <QVector>
#include <QTime>
#include "ui_MavlinkInspector.h"
#include "ardupilotmega/mavlink.h"
#include "common/mavlink.h"
#include "uAvionix/mavlink.h"
#include "DroneInfo.h"


class MavlinkInspector : public QWidget
{
	Q_OBJECT

public:
	MavlinkInspector(QWidget *parent = Q_NULLPTR);
	~MavlinkInspector();
	void Notify(uint8_t sysid, uint8_t msgid, uint8_t*data);
	uint32_t Get_sys_time();
	
	QString Translate_id(uint8_t msgid);
	QVector<QStringList> Translate_data(uint8_t msgid, uint8_t* data);
	float GetFrequency(uint8_t msgid);
	QVector<QString> Get_id_list();
	DroneInfo Get_drone_info(QString sysname);

private:
	Ui::MavlinkInspector ui;
	QTime time;
	float Interval[256];
	uint32_t boot_time_ms;

};
