#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "IMU.h"
#include "Sensors.h"

void getEstimatedAttitude();

void computeIMU() {
	uint8_t axis;
	static int16_t gyroADCprevious[3] = { 0, 0, 0 };
	int16_t gyroADCp[3];
	int16_t gyroADCinter[3];
	static uint32_t timeInterleave = 0;

	//we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
	//gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
	ACC_getADC();
	Gyro_getADC();
	for (axis = 0; axis < 3; axis++)
	{
		gyroADCp[axis] = imu.gyroADC[axis];
	}
	timeInterleave = micros();
	annexCode();
	uint8_t t = 0;
	while ((uint16_t) (micros() - timeInterleave) < 650)
	{
		t = 1; //empirical, interleaving delay between 2 consecutive reads
	}
	if (!t)
	{
		annex650_overrun_count++;
	}
	Gyro_getADC();
	//mwm: some sort of low pass filter...
	for (axis = 0; axis < 3; axis++) {
		gyroADCinter[axis] = imu.gyroADC[axis] + gyroADCp[axis];
		// empirical, we take a weighted value of the current and the previous values
		imu.gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis]) / 3;
		gyroADCprevious[axis] = gyroADCinter[axis] >> 1;
	}
}

void getEstimatedAttitude() {
	//mwm: deleted everything
}

uint8_t getEstimatedAltitude() {
	//mwm: deleted everything
	return 1;
}

