#pragma once
#include "Sensor.h"

using namespace Eigen;

class Mapping
{
public:
	Mapping(Map<MatrixXd> currentgrid, double* params);
	void Nav(Vector3d pose, MatrixXd obinfo, MatrixXd landinfo, Sensor senObject);
	void Grid(MatrixXd rnCN, MatrixXd RnCN, MatrixXd obinfo, MatrixXd cells); 
	void MeasureObs(); 
	void MeasureLand();
	
	VectorXd SubToInd(MatrixXd cells);	// Convert to Linear Index
	MatrixXd IndToSub(VectorXd scan);	// Convert to XY Grid Co-od

	MatrixXd GetGrid(bool flag);
//	VectorXd GetSparseVect(bool flag); //Replace GetObsVect and GetLandVect
//	VectorXd GetObsVect();
//	VectorXd GetLandVect();

private:
	double dec;
	double inc;
	double max;
	double min;
	double res;
	double thres;
	double width; //Not initialised Yet
	double length; //Not initialised Yet
	MatrixXd grid;
	MatrixXd gridy;
	VectorXd y;
	VectorXd yhat;
};

