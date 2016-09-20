#pragma once
#include "Sensor.h"

using namespace Eigen;

class Mapping
{
public:
	Mapping(MatrixXd currentgrid, double* params);
	void Nav(Vector3d pose, MatrixXd obinfo, MatrixXd landinfo, Sensor senObject);
	void Grid(MatrixXd rnCN, MatrixXd RnCN, MatrixXd obinfo, VectorXi camera, MatrixXi cells);

	RowVectorXi IsMember(RowVectorXi set, RowVectorXi subset);
	RowVectorXi LogicalIndex(RowVectorXd elem, const double cond, bool flag);

	MatrixXd GetGrid(bool flag);


	//	VectorXd GetSparseVect(bool flag); //Replace GetObsVect and GetLandVect
	//	VectorXd GetObsVect();
	//	VectorXd GetLandVect();

	//void MeasureObs(); 	
	void MeasureLand(MatrixXd rnCN, MatrixXd RnCN, Vector3d rnBN, Matrix3d RnBN, MatrixXd LC, VectorXd cam, MatrixXd lmrks);

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

