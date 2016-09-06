#include "Mapping.h"

Mapping::Mapping(Map<MatrixXd> currentgrid, double* params)
{
	grid = currentgrid;
	dec = params[0];
	inc = params[1];
	max = params[2];
	min = params[3];
	res = params[4];
	thres = params[5];
}

void Mapping::Nav() {
	
}
/*void Mapping::Grid(MatrixXd rnCN, MatrixXd RnCN, MatrixXd QC, VectorXd ocam, VectorXd radii, int numObstacles) {
	//MatrixXd rnCN(3, 4), MatrixXd RnCN(3, 12), MatrixXd QC(4, 255), VectorXd ocam(255), VectorXd radii(255)
	// Decided on 255 -> Large Number to hopefully contain all obstacles seen in timestep. (Can be larger, or make dynamic)
	// Have int index telling us how big this matrices are for current timestep (from length of camera data) (nuObstacles)
}*/
void Mapping::MeasureObs() {
	//Takes in all
}
void Mapping::MeasureLand() {
	// FILL ME IN FIRST!!!
}

MatrixXd Mapping::GetGrid() {
	return grid;
}

VectorXd Mapping::GetObsVect() {
	return y;
}

VectorXd Mapping::GetLandVect() {
	return yhat;
}
