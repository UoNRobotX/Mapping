#include <iostream>
#include <mex.h>
#include "Mapping.h"

using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	/**********************************************************************************
	* The Main Function() formats Matlab Parameters and Executes C++ Code.            *
	***********************************************************************************/

	Map<MatrixXd> gridhist(mxGetPr(prhs[0]), mxGetM(prhs[0]), mxGetN(prhs[0]));
	Mapping MapObject((MatrixXd)gridhist, mxGetPr(prhs[1]));

	Sensor SenObject(mxGetPr(prhs[2]));
	Map<MatrixXd> data(mxGetPr(prhs[3]), mxGetM(prhs[3]), mxGetN(prhs[3]));
	Map<VectorXd> camera(mxGetPr(prhs[4]), mxGetM(prhs[4]));

	// Will Not Need when Nav Complete!!

	Map<MatrixXd> rnCN(mxGetPr(prhs[5]), 3, SenObject.GetCameras());
	Map<MatrixXd> RnCN(mxGetPr(prhs[6]), 3, SenObject.GetCameras() * 3);
	Map<MatrixXd> cells(mxGetPr(prhs[7]), mxGetM(prhs[7]), mxGetN(prhs[7]));

	/*Call C++ Functions*/

	//MapObject.Nav(pose, obinfo, landinfo, senObject);
	MapObject.Grid((MatrixXd)rnCN, (MatrixXd)RnCN, (MatrixXd)data, (VectorXi)camera.cast<int>(), (MatrixXi)cells.cast<int>());
	MapObject.MeasureLand(rnCN,RnCN,rnBN,RnBN,LC,cam,lmrks);

	/*Outputs Requested by Matlab*/

	MatrixXd grid = MapObject.GetGrid(0);
	plhs[0] = mxCreateDoubleMatrix((mwSize)grid.rows(), (mwSize)grid.cols(), mxREAL);
	Map<MatrixXd>(mxGetPr(plhs[0]), grid.rows(), grid.cols()) = grid;

	MatrixXd gridy = MapObject.GetGrid(1);
	plhs[1] = mxCreateDoubleMatrix((mwSize)gridy.rows(), (mwSize)gridy.cols(), mxREAL);
	Map<MatrixXd>(mxGetPr(plhs[0]), gridy.rows(), gridy.cols()) = gridy;
}