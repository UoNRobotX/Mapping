#pragma once

#include <Eigen/Dense>

using namespace Eigen;

class Mapping
{
public:
	Mapping(double* params);
	void Nav();
	void Grid(MatrixXd rnCN, MatrixXd RnCN, MatrixXd QC,VectorXd ocam, VectorXd radii, int numObstacles);
	void MeasureObs(); 
	void MeasureLand();
	MatrixXd GetGrid();
	VectorXd GetObsVect();
	VectorXd GetLandVect();

private:
	double dec;
	double inc;
	double max;
	double min;
	double thres;
	double res;
	double width;
	double length;
	double lrad;
	Vector3f orad;
	MatrixXd grid;
};

