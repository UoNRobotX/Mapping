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

	/*Obtain All Other Variables from Matlab Utilsing Sensor Information*/ // Add Robustness with Sensor Class

	Sensor SenObject(mxGetPr(prhs[2]));
	Map<MatrixXd> rnCN(mxGetPr(prhs[3]), 3, SenObject.GetCameras());
	Map<MatrixXd> RnCN(mxGetPr(prhs[4]), 3, SenObject.GetCameras()*3);
	Map<MatrixXd> obinf(mxGetPr(prhs[5]), mxGetM(prhs[4]), SenObject.GetMax());
	Map<MatrixXd> cells(mxGetPr(prhs[6]), mxGetM(prhs[5]), mxGetN(prhs[5]));

	/*Call C++ Functions*/

//	MapObject.Nav(pose, obinfo, landinfo, senObject);

	MapObject.Grid(rnCN, RnCN, obinf, cells);


	/*Outputs for Grid Function*/

	grid = MapObject.GetGrid(0);
	gridy = MapObject.GetGrid(1);
	plhs[0] = mxCreateDoubleMatrix((mwSize)grid.rows(), (mwSize)grid.cols(), mxREAL);
	Map<MatrixXd>(mxGetPr(plhs[0]), grid.rows(), grid.cols()) = grid;
	

	
}