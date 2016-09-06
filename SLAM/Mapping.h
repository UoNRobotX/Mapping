#pragma once
#include <Eigen/Dense>

using namespace Eigen;

class Mapping
{
public:
	Mapping(Map<MatrixXd> currentgrid, double* params);

	void Nav();
	void Grid(MatrixXd rnCN, MatrixXd RnCN, MatrixXd QC,RowVectorXd cam, RowVectorXd radii, RowVectorXd );
	void MeasureObs(); 
	void MeasureLand();
	MatrixXd GetGrid(bool flag);
	VectorXd GetObsVect();
	VectorXd GetLandVect();

private:
	double dec;
	double inc;
	double max;
	double min;
	double res;
	double thres;
	//double width;
	//double length;
	MatrixXd grid;
	MatrixXd gridy;
	VectorXd y;
	VectorXd yhat;
};

