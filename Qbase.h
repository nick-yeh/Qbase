#pragma once


#include <QtWidgets/QMainWindow>
#include "ui_Qbase.h"
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QLabel>
#include <QTimer>
#include "checkcrc.h"
#include "MavlinkProcess.h"
#include "MavlinkInspector.h"
#include "MavlinkCommand.h"

class QWebEngineView;
class Qbase : public QMainWindow
{
	Q_OBJECT
public:
	void UpdateSerialInfo();
	void UpdateDroneID();
	void UpdateDroneInfo();
	void UpdateMapMarker();
	
	//-------------
	//void webShow(const QString &url);
	//------------

private slots:
	void on_openPortBtn_clicked();	//利用“转到槽”添加的槽函数声明
	void on_openMavlinkInspector_Btn_clicked();
	void on_set_mode_btn_clicked();
	void on_openTestBtn_clicked();
	void on_openArmBtn_clicked();
	void on_update();
	void on_landBtn_clicked();
	void on_disarmBtn_clicked();
	void on_flyNorthBtn_clicked();
	void on_flySouthBtn_clicked();
	void on_flyWestBtn_clicked();
	void on_flyEastBtn_clicked();
	void on_flyUpBtn_clicked();
	void on_flyDownBtn_clicked();
	void send_guide_attitude();
	void read_Com();			//手动添加的槽函数声明，用于读出串口缓冲区的内容
	void on_flyToHereBtn_clicked();
	void on_takeOffBtn_clicked();
	void on_setMarkerOnMapBtn_clicked();


	//----------------------------------------------
	void mapLoadFinished();
	void clearAllMarker();
	void setOneMarker();
	//----------------------------------------------


public:
	Qbase(QWidget *parent = Q_NULLPTR);


private:
	Ui::QbaseClass ui;
	QSerialPort m_serial;
	QTimer *p_timer = new QTimer(this);
	QTimer m_timer;
	QTimer m_refresh_timer;
	MavlinkProcess m_mp;
	MavlinkInspector m_mi;
	MavlinkCommand m_md;
	uint8_t count;

	//-----------------
	QWebEngineView *view;
	//-------------------

};
