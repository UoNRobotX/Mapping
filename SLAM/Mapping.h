#pragma once
#include "Sensor.h"

using namespace Eigen;

class Mapping
{
public:
	Mapping(MatrixXd currentgrid, double* params);
	void Update(VectorXd pose, VectorXi obscam, VectorXi lndcam, MatrixXd obsinfo, MatrixXd lndinfo, Sensor senObject);
	void Grid(MatrixXd rnCN, MatrixXd RnCN, MatrixXd obsinfo, VectorXi obscam, MatrixXi cells);
	RowVectorXi LogicalIndex(RowVectorXd uelem, RowVectorXd lelem, const double upper, const double lower);
	MatrixXd GetGrid(bool flag);
	VectorXd GetVect(bool flag);
		
	void MeasureLand(MatrixXd rnCN, MatrixXd RnCN, Vector3d rnBN, Matrix3d RnBN, MatrixXd LC, RowVectorXd cam, MatrixXd lmrks);
	//void MeasureObs();

private:
	double res;
	double dec;
	double inc;
	double thres;
	double maxval;
	double minval;
	VectorXd y;
	VectorXd yhat;
	MatrixXd grid;
	MatrixXd gridy;
};

