#pragma once
#include <Eigen/Dense>

using namespace Eigen;

class Sensor
{
public:
	Sensor(double* params);
	int GetCameras();
private:
	int cameras;
	double height;
	MatrixXd rbCB;
	MatrixXd RbCB;
	// Add in Sensor Covariances Etc!!
};

