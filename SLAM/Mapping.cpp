#include "Mapping.h"

Mapping::Mapping(Map<MatrixXd> currentgrid, double* params)
{
	grid = currentgrid;
	gridy = MatrixXd::Zero(grid.rows(), grid.cols());
	dec = params[0];
	inc = params[1];
	max = params[2];
	min = params[3];
	res = params[4];
	thres = params[5];
}

void Mapping::Nav(Vector3d pose, MatrixXd obinfo, MatrixXd landinfo, Sensor senObject) {


}


void Mapping::Grid(MatrixXd rnCN, MatrixXd RnCN, MatrixXd obinfo, MatrixXd cells) {
	MatrixXd rcQC(3, obinfo.cols());
	rcQC = obinfo.topRows(2);
//	obstacles = rcQC.colwise().sum();
}


void Mapping::MeasureObs() {
	//Takes in all
}


/* DESCRIPTION: This function returns landmark bearings and ranges, both from the camera coordinates and dead reckoning
 * INPUTS:  rnCN - 3x4         (NED by number of cameras)
 *	        RnCN - 3x12        (3x3 rotation matrices concatenated together)
 *	        rnBN - 3x1         (NED position of boat in world coordinates)
 *	        RnBn - 3x3         (Rotation of the boat)
 *	        LC   - 4xObsL      (x,y,z,theta(radius of bouy) from camera)
 *	        cam  - 1xObsL      (This camera observed this measurement)
 *	        lmrks- 4x8         (STATIC NED GPS coordinates for landmarks and a standard height for every landmark)
 * OUTPUTS: y    - 1x(2*ObsL)  (rangey,bearingy)
 *	        yhat - 1x(2*ObsL)  (rangeyhat,bearingyhat)
 */
void Mapping::MeasureLand(MatrixXd rnCN, MatrixXd RnCN, Vector3d rnBN, Matrix3d RnBN, MatrixXd LC, RowVectorXd cam, MatrixXd lmrks) {


	
	
	
	
	
	/*********************************************************************************************************************/
	/* Camera Observation */
	
	if (LC.rows() > 1) {  // This condition assumes that LC will be initialised to a 1x1 matrix if there is no landmarks
		MatrixXd rcLC = LC.topRows(3);
		MatrixXd thLC = LC.bottomRows(1);
		int lnd = rcLC.cols();
		VectorXd bearingy(lnd),
				 bearingyhat(lnd),
				 rangey(lnd),
				 rangeyhat(lnd);
		MatrixXd temp(3, 1);
		temp << 2, 1, 0;
		MatrixXd ind = 3 * cam.replicate(3, 1) - temp.replicate(1, lnd);
		MatrixXd temp3 = thLC.array().sin();
		MatrixXd temp2 = lmrks.row(3).array().cwiseQuotient(temp3.array());
		MatrixXd rcPC(3, lnd);
		rcPC = temp2.replicate(3, 1).cwiseProduct(rcLC);
		MatrixXd rnPC = MatrixXd::Zero(3, lnd);
		MatrixXd rnPN(3, lnd);
		
		for (int count = 0; count < lnd; count++) {
			rnPN.col(count) = RnCN.middleCols((cam(count) - 1) * 3, 3)*rcPC.col(count) + rnCN.col(cam(count) - 1);                 //.col(ind.col(count))*rcPC.col(count); // be careful with the use of count. Not sure the 0/1 starting difference issue is resovled in this line
		}
		
		
		MatrixXd diff = rnPN - rnBN.replicate(1, lnd);
		for (int count = 0; count < lnd; count++) {
			bearingy(count) = atan2(diff(1, count), diff(0, count));
		}
		rangey = diff.colwise().norm();
		
		/**********************************************************************************************************************/
		/* GPS and IMP Approximation with Assumed GPS Landmark Location */

		MatrixXd local = RnBN*(lmrks.topRows(3) - rnBN.replicate(1, lnd));
		
		for (int count = 0; count < lnd; count++) {
			bearingy(count) = atan2(local(1, count), local(0, count));
		}
		rangeyhat = local.colwise().norm();
		
		
		/**********************************************************************************************************************/
		/* Form Outputs */
		y.resize(lnd * 2);
		yhat.resize(lnd * 2);
		y << rangey,bearingy;
		yhat << rangeyhat,bearingyhat;
	}
	else { // Using a 1 element array with a zero in it to indicate there was no observations
		y.resize(1);
		yhat.resize(1);
		y << 0;
		yhat << 0;
	}
	
}

MatrixXd Mapping::GetGrid(bool flag) {
	MatrixXd output;
	output = (flag) ? gridy : grid;
	return output;
}

VectorXd Mapping::GetObsVect(bool flag) {
	VectorXd output;
	output = (flag) ? yhat : y;
	return output;
}

/*
VectorXd Mapping::GetLandVect() {
	return yhat;
}
*/


VectorXd Mapping::SubToInd(MatrixXd cells) {
	VectorXd v(2);
	return v;
	// Convert to Linear Index
}
MatrixXd Mapping::IndToSub(VectorXd scan) {
	// Convert to XY Grid Co-od
	MatrixXd v(2,2);
	return v;
}

/*

VectorXd Mapping::GetSparseVect(bool flag) {
	//Replace Get""Vect
}

*/