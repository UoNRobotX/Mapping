#pragma once
#include <Eigen/Dense>

using namespace Eigen;

class Sensor
{
public:
	Sensor(MatrixXd vects, MatrixXd rotations, double* params);
	double GetRange();
	double GetHeight();	
	MatrixXd GetVects();
	MatrixXd GetRotations();
private:
	double range;
	double height;
	MatrixXd rbCB;
	MatrixXd RbCB;
};

