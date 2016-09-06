#include <iostream>
#include <mex.h>
#include "Mapping.h"

using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	/*Initialise Mapping Class Parameters*/

	MatrixXd outgrid;
	Map<MatrixXd> gridcurrent(mxGetPr(prhs[0]), mxGetM(prhs[0]), mxGetN(prhs[0]));
	Mapping MapObject(gridcurrent, mxGetPr(prhs[1]));

	/*Obtain All Other Variables from Matlab*/

	/*Call C++ Functions*/

	/*Outputs*/

	outgrid = MapObject.GetGrid();
	plhs[0] = mxCreateDoubleMatrix((mwSize)outgrid.rows(), (mwSize)outgrid.cols(), mxREAL);
	Map<MatrixXd>(mxGetPr(plhs[0]), outgrid.rows(), outgrid.cols()) = outgrid;
	

	
	
}