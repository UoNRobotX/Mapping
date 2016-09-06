#pragma once
#include <Eigen/Dense>

using namespace Eigen;

class Sensor
{
public:
	Sensor(double* params);
	int GetCameras();
	int GetMax();
private:
	int cameras;
	int maxobserve;
	double height;
	MatrixXd rbCB;
	MatrixXd RbCB;
};

