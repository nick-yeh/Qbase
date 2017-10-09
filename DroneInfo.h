#pragma once
#include "math.h"

class DroneInfo
{
public:
	int altitude;
	int heading;
	double latitude;
	double longitude;
	int relative_alt;
	long timestamp;
	int vx;
	int vy;
	int vz;

public:
	DroneInfo();
	~DroneInfo();
};

