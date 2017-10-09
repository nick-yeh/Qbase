#pragma once

#include "MavlinkInspector.h"
#include "ardupilotmega/mavlink.h"
#include "common/mavlink.h"
#include "uAvionix/mavlink.h"


class MavlinkProcess
{
public:
	MavlinkProcess();
	~MavlinkProcess();
	void setInspector(MavlinkInspector* in);
	int sys_id;
	int com_id;

public:
	void Parse(uint8_t* msg);
	MavlinkInspector* p_inspector;
	uint8_t Get_sys_id();
	uint8_t Get_com_id();
};

