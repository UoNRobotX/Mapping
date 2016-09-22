#include <iostream>
#include <mex.h>
#include "Mapping.h"

using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	/*Obtain Data from Matlab*/

	Map<VectorXd> pose(mxGetPr(prhs[0]), mxGetM(prhs[0]));
	Map<VectorXd> obscam(mxGetPr(prhs[1]), mxGetM(prhs[1]));
	Map<VectorXd> lndcam(mxGetPr(prhs[2]), mxGetM(prhs[2]));
	Map<MatrixXd> obsinfo(mxGetPr(prhs[3]), mxGetM(prhs[3]), mxGetN(prhs[3]));
	Map<MatrixXd> lndinfo(mxGetPr(prhs[4]), mxGetM(prhs[4]), mxGetN(prhs[4]));
	Map<MatrixXd> gridhist(mxGetPr(prhs[5]), mxGetM(prhs[5]), mxGetN(prhs[5]));
	Map<MatrixXd> rbCB(mxGetPr(prhs[6]), mxGetM(prhs[6]), mxGetN(prhs[6]));
	Map<MatrixXd> RbCB(mxGetPr(prhs[7]), mxGetM(prhs[7]), mxGetN(prhs[7]));
	
	/*Initalise Parameters in Classes Mapping and Sensor*/

	Mapping MapObject((MatrixXd)gridhist, mxGetPr(prhs[8]));
	Sensor SenObject((MatrixXd)rbCB, (MatrixXd)RbCB, mxGetPr(prhs[9]));

	/*Call C++ Function to Update Required Information*/


	MapObject.Update((VectorXd)pose, (VectorXi)obscam.cast<int>(), (VectorXi)lndcam.cast<int>(), (MatrixXd)obsinfo, (MatrixXd)lndinfo, SenObject);

        /*Daniels
	Map<MatrixXd> rnCN(mxGetPr(prhs[5]), 3, SenObject.GetCameras());
	Map<MatrixXd> RnCN(mxGetPr(prhs[6]), 3, SenObject.GetCameras() * 3);
	Map<MatrixXd> cells(mxGetPr(prhs[7]), mxGetM(prhs[7]), mxGetN(prhs[7]));
	Map<Vector3d> rnBN(mxGetPr(prhs[8]), 3);
	Map<Matrix3d> RnBN(mxGetPr(prhs[9]));
	Map<MatrixXd> LC(mxGetPr(prhs[10]), 4, mxGetN(prhs[10]));
	Map<RowVectorXd> camL(mxGetPr(prhs[11]), mxGetN(prhs[11]));
        Map<MatrixXd> lmrks(mxGetPr(prhs[12]), 4, mxGetN(prhs[12]));*/


	/*Request Outputs for Matlab*/

	MatrixXd grid = MapObject.GetGrid(0);
	//VectorXd y = MapObject.GetVect(0);
	//VectorXd yhat = MapObject.GetVect(1);

	/*Outputs Requested by Matlab*/

        //MapObject.Grid((MatrixXd)rnCN, (MatrixXd)RnCN, (MatrixXd)data, (VectorXi)camera.cast<int>(), (MatrixXi)cells.cast<int>());
        //MapObject.MeasureLand(rnCN, RnCN, rnBN, RnBN, LC, camL, lmrks);
	
	MatrixXd grid = MapObject.GetGrid(0);
	plhs[0] = mxCreateDoubleMatrix((mwSize)grid.rows(), (mwSize)grid.cols(), mxREAL);
	//plhs[1] = mxCreateDoubleMatrix((mwSize)y.rows(), (mwSize)y.cols(), mxREAL);
	//plhs[2] = mxCreateDoubleMatrix((mwSize)yhat.rows(), (mwSize)yhat.cols(), mxREAL);
        Map<MatrixXd>(mxGetPr(plhs[0]), grid.rows(), grid.cols()) = grid;
	//Map<VectorXd>(mxGetPr(plhs[1]), y.rows()) = y;
	//Map<VectorXd>(mxGetPr(plhs[2]), yhat.rows()) = yhat;

        /*Daniels
	MatrixXd gridy = MapObject.GetGrid(1);
	plhs[1] = mxCreateDoubleMatrix((mwSize)gridy.rows(), (mwSize)gridy.cols(), mxREAL);
	Map<MatrixXd>(mxGetPr(plhs[1]), gridy.rows(), gridy.cols()) = gridy;
	
	VectorXd y = MapObject.GetObsVect(0);
	plhs[2] = mxCreateDoubleMatrix((mwSize)y.rows(),1, mxREAL);
	Map<VectorXd>(mxGetPr(plhs[2]), y.rows()) = y;

	VectorXd yhat = MapObject.GetObsVect(1);
	plhs[3] = mxCreateDoubleMatrix((mwSize)yhat.rows(),1, mxREAL);
        Map<VectorXd>(mxGetPr(plhs[3]), yhat.rows()) = yhat;*/
}
