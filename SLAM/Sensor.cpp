#include "Sensor.h"

Sensor::Sensor(double* params){
	cameras = (int)params[0];
	maxobserve = (int)params[1];
	height = params[2];
}

int Sensor::GetCameras() {
	return cameras;
}

int Sensor::GetMax() {
	return maxobserve;
}