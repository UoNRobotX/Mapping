#include "Sensor.h"

Sensor::Sensor(double* params){
	cameras = (int)params[0];
	height = params[1];
}

int Sensor::GetCameras() {
	return cameras;
}