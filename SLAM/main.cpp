#include <iostream>
#include <mex.h>
#include "Mapping.h"

using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	/*Initialise Mapping Class Parameters*/

	MatrixXd grid, gridy;
	Map<MatrixXd> gridcurrent(mxGetPr(prhs[0]), mxGetM(prhs[0]), mxGetN(prhs[0]));
	Mapping MapObject(gridcurrent, mxGetPr(prhs[1]));

	/*Obtain All Other Variables from Matlab*/

	Map<MatrixXd> rnCN(mxGetPr(prhs[2]), 3, 4);
	Map<MatrixXd> RnCN(mxGetPr(prhs[3]), 3, 12);
	Map<MatrixXd> QC(mxGetPr(prhs[4]), mxGetM(prhs[4]), mxGetN(prhs[4]));
	Map<MatrixXd> cells(mxGetPr(prhs[5]), mxGetM(prhs[4]), mxGetN(prhs[5]));
	Map<RowVectorXd> cam(mxGetPr(prhs[6]), mxGetN(prhs[6]));
	//Map<RowVectorXd> scan(mxGetPr(prhs[7]), mxGetN(prhs[7])); DONT NEED THIS!

	/*Call C++ Functions*/



	/*Outputs for Grid Function*/

	grid = MapObject.GetGrid(0);
	gridy = MapObject.GetGrid(1);
	plhs[0] = mxCreateDoubleMatrix((mwSize)grid.rows(), (mwSize)grid.cols(), mxREAL);
	Map<MatrixXd>(mxGetPr(plhs[0]), grid.rows(), grid.cols()) = grid;
	

	
}