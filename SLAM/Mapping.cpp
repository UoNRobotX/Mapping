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


/* DESCRIPTION This function returns landmark bearings and ranges, both from the camera coordinates and dead reckoning
 * INPUTS:  rnCN - 
 *		    RnCN -
 *		    rnBN -
 *		    RnBN -
 *		    LC   -
 *		    cam  -
 *		    lmrks-
 * OUTPUTS: y    -
 *			yhat -
 */
void Mapping::MeasureLand(/*rnCN,RnCN,rnBN,RnBN,LC,cam,lmrks*/) {

	MatrixXd LC(1,1); // Initialisation value of LC to detect when there are no landmark observations
	
	VectorXd bearingy,
			 bearingyhat,
			 rangey,
			 rangeyhat;
	/*********************************************************************************************************************/
	/* Camera Observation */

	if (LC.rows() > 1) {  // This condition assumes that LC will be initialised to a 1x1 matrix if there is no landmarks
		MatrixXd rcLC = LC.topRows(3);
		VectorXd thLC = LC.bottomRows(1);
		int lnd = rcLC.rows();
		MatrixXd temp(3, 1);
		temp << 2, 1, 0;
		MatrixXd ind = 3 * cam.replicate(3, 1) - temp.replicate(1, lnd);
		MatrixXd temp2 = -lmrks.row(3).cwiseQuotent(thLC.sin());					
		MatrixXd rcPC(3, lnd);
		rcPC = temp2.replicate(3, 1)*rcLC;
		MatrixXd rnPC = MatrixXd::Zero(3, lnd);
		for (int count = 0; count < lnd; count++) {
			rnPC.col(count) = RnCN.col(ind.col(count))*rcPC.col(count); // be careful with the use of count. Not sure the 0/1 starting difference issue is resovled in this line
		}
		VectorXd rnPN = rnPC + rnCN.col(cam);	// careful! does cam need to have a -1?? //Is it a vector, or must it be a matrix??
		MatrixXd diff = rnPN - rnBN.replicate(1, lnd);
		bearingy = atan2(diff.row(1), diff.row(0));    // unsure how atan2 is going to handle this
		rangey = diff.square().sum().sqrt();
	}

	/**********************************************************************************************************************/
	/* GPS and IMP Approximation with Assumed GPS Landmark Location */

	if (lmrks.rows() > 1) {  // This condition assumes that lmrks will be initialised to a 1x1 matrix if there is no landmarks
		MatrixXd local = RnBN*(lmrks - rnBN.replicate(1, lmrks.rows()));
		bearingyhat = atan2(local.row(1), local.row(0));  // unsure how atan2 is going to handle this
		rangeyhat = local.square().sum().sqrt();     
	}

	/**********************************************************************************************************************/
	/* Form Outputs */

	if (lmrks.rows() > 1) {
		VectorXd y(rangey.rows(),rangey.cols()+bearingy.cols());
		VectorXd yhat(rangeyhat.rows(),rangeyhat.cols()+bearingyhat.cols());
		y << rangey,bearingy;
		yhat << rangeyhat,bearingyhat;
	}
	else { // Using a 1 element array with a zero in it to indicate there was no observations
		VectorXd y(1,1);
		VectorXd yhat(1,1);
		y << 0;
		yhat << 0;

	}
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
