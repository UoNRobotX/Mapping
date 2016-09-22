#include "Sensor.h"

Sensor::Sensor(MatrixXd vects, MatrixXd rotations, double* params){
	rbCB = vects; RbCB = rotations;
	range = params[0]; height = params[1];
}

double Sensor::GetRange() {
	return range;
}

double Sensor::GetHeight() {
	return height;
}

MatrixXd Sensor::GetVects() {
	return rbCB;
}

MatrixXd Sensor::GetRotations() {
	return RbCB;
}