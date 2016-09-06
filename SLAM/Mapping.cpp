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
 *	        RnCN - 3x12        (3x3 rotation matrices comcatenated together)
 *	        rnBN - 3x1         (NED position of boat in world coordinates)
 *	        RnBn - 3x3         (Rotation of the boat)
 *	        LC   - 4xObsL      (x,y,z,theta(radius of bouy) from camera)
 *	        cam  - 1xObsL      (This camera observed this measurement)
 *	        lmrks- 3x8         (STATIC NED GPS coordinates for landmarks)
 * OUTPUTS: y    - 1x(2*ObsL)  (rangey,bearingy)
 *	        yhat - 1x(2*ObsL)  (rangeyhat,bearingyhat)
 */
void Mapping::MeasureLand(MatrixXd rnCN, MatrixXd RnCN, Vector3d rnBN, Matrix3d RnBN, MatrixXd LC, VectorXd cam, MatrixXd lmrks) {
 	
	VectorXd y(4);
	VectorXd yhat(4);
	y << 1, 2, 3, 4;
	yhat << 5, 6, 7, 8;
	
	
	
	/*
 	
	MatrixXd LC(1,1); // Initialisation value of LC to detect when there are no landmark observations
	
	VectorXd bearingy,
			 bearingyhat,
			 rangey,
			 rangeyhat;
	/*********************************************************************************************************************/
	/* Camera Observation */
	/*
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
	/*
	if (lmrks.rows() > 1) {  // This condition assumes that lmrks will be initialised to a 1x1 matrix if there is no landmarks
		MatrixXd local = RnBN*(lmrks - rnBN.replicate(1, lmrks.rows()));
		bearingyhat = atan2(local.row(1), local.row(0));  // unsure how atan2 is going to handle this
		rangeyhat = local.square().sum().sqrt();     
	}

	/**********************************************************************************************************************/
	/* Form Outputs */
	/*
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


	}*/
}

MatrixXd Mapping::GetGrid(bool flag) {
	MatrixXd output;
	output = (flag) ? gridy : grid;
	return output;
}

/*
VectorXd Mapping::GetObsVect() {
	return y;
}

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