#pragma once
#include "Sensor.h"

using namespace Eigen;

class Mapping{
public:
	Mapping(MatrixXd currentgrid, MatrixXd markers, double* params);
	void Update(VectorXd pose, VectorXi obscam, VectorXi lndcam, MatrixXd obsinfo, MatrixXd lndinfo, Sensor senObject);
	void Grid(MatrixXd rnCN, MatrixXd RnCN, MatrixXd obsinfo, VectorXi obscam, MatrixXi cells);
	void ObstVects(Vector2d rnBN, MatrixXi cells, double range);
	void LandVects(MatrixXd rnCN, MatrixXd RnCN, Vector3d rnBN, Matrix3d RnBN, MatrixXd lndinfo, VectorXi lndcam);
	RowVectorXi LogicalIndex(RowVectorXd uelem, RowVectorXd lelem, const double upper, const double lower);
	MatrixXd GetGrid(bool flag);
	VectorXd GetVect(bool flag);
private:
	double res;
	double dec;
	double inc;
	double thres;
	double maxval;
	double minval;
	VectorXd ylnd, ylndhat;
	VectorXd yobs, yobshat;
	MatrixXd grid, gridy;
	MatrixXd landmarks;
};

