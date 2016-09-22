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
	Map<Vector3d> rnBN(mxGetPr(prhs[8]), 3);
	Map<Matrix3d> RnBN(mxGetPr(prhs[9]));
	Map<MatrixXd> LC(mxGetPr(prhs[10]), 4, mxGetN(prhs[10]));
	Map<RowVectorXd> camL(mxGetPr(prhs[11]), mxGetN(prhs[11]));
	Map<MatrixXd> lmrks(mxGetPr(prhs[12]), 4, mxGetN(prhs[12]));

	/*Call C++ Functions*/

	//MapObject.Nav(pose, obinfo, landinfo, senObject);
	MapObject.Grid((MatrixXd)rnCN, (MatrixXd)RnCN, (MatrixXd)data, (VectorXi)camera.cast<int>(), (MatrixXi)cells.cast<int>());
	MapObject.MeasureLand(rnCN, RnCN, rnBN, RnBN, LC, camL, lmrks);

	/*Outputs Requested by Matlab*/
	
	MatrixXd grid = MapObject.GetGrid(0);
	plhs[0] = mxCreateDoubleMatrix((mwSize)grid.rows(), (mwSize)grid.cols(), mxREAL);
	Map<MatrixXd>(mxGetPr(plhs[0]), grid.rows(), grid.cols()) = grid;

	MatrixXd gridy = MapObject.GetGrid(1);
	plhs[1] = mxCreateDoubleMatrix((mwSize)gridy.rows(), (mwSize)gridy.cols(), mxREAL);
	Map<MatrixXd>(mxGetPr(plhs[1]), gridy.rows(), gridy.cols()) = gridy;
	
	VectorXd y = MapObject.GetObsVect(0);
	plhs[2] = mxCreateDoubleMatrix((mwSize)y.rows(),1, mxREAL);
	Map<VectorXd>(mxGetPr(plhs[2]), y.rows()) = y;

	VectorXd yhat = MapObject.GetObsVect(1);
	plhs[3] = mxCreateDoubleMatrix((mwSize)yhat.rows(),1, mxREAL);
	Map<VectorXd>(mxGetPr(plhs[3]), yhat.rows()) = yhat;
	
}