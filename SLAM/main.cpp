#include <iostream>
#include <mex.h>
#include <Eigen/Dense>

void placeElement(double *el, double *er, int x);

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	double *el, *er;
	int element;
	size_t rows, cols;

	rows = mxGetM(prhs[0]);
	cols = mxGetN(prhs[0]);
	element = (int)*mxGetPr(prhs[1]);
	
	plhs[0] = mxCreateDoubleMatrix((mwSize)rows, (mwSize)cols, mxREAL);
	plhs[1] = mxCreateDoubleScalar(element);

	el = mxGetPr(plhs[0]);
	er = mxGetPr(prhs[0]);
	
	placeElement(el, er, element);	
}

void placeElement(double *el, double *er, int x) {
	el[0] = er[x];
}