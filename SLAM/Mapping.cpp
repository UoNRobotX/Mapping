#include "Mapping.h"

/*--------------------------------------------------------------------------------------------------------------------------------------*/

Mapping::Mapping(MatrixXd currentgrid, double* params) {

	res = params[0]; dec = params[1];
	inc = params[2]; thres = params[3];
	maxval = params[4]; minval = params[5];
	grid = currentgrid; gridy = MatrixXd::Zero(grid.rows(), grid.cols());
}

/*--------------------------------------------------------------------------------------------------------------------------------------*/

void Mapping::Update(VectorXd pose, VectorXi obscam, VectorXi lndcam, MatrixXd obsinfo, MatrixXd lndinfo, Sensor senObject) {

	/*Memory Allocation*/
	
	Matrix3d RnBN;
	Vector3d rnBN;
	MatrixXi cells;
	RowVectorXi index;
	RowVectorXd magnitude, nrth, east, constant;
	Vector2d gridpose, minlim, maxlim;
	MatrixXd rnCN, RnCN, subscripts;
	size_t nrthlen, eastlen;
	
	/*Parameter Initialisation*/
	double range = senObject.GetRange() / res;
	double roll = pose(3); double pitch = pose(4); double yaw = pose(5);
	Vector2d maxsub((const double)(grid.rows()-1), (const double)(grid.cols()-1));
	Vector2d minsub = Vector2d::Zero();

	/*Global Camera Positions and Rotations*/
	rnBN << pose(0), pose(1), cos(pitch)*cos(roll)*senObject.GetHeight();
	RnBN << cos(yaw)*cos(pitch), -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll), sin(yaw)*sin(roll) + cos(yaw)*cos(roll)*sin(pitch),
		sin(yaw)*cos(pitch), cos(yaw)*cos(roll) + sin(roll)*sin(pitch)*sin(yaw), -cos(yaw)*sin(roll) + sin(yaw)*cos(roll)*sin(pitch),
		-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll);
	rnCN = (RnBN*senObject.GetVects()).colwise() + rnBN;
	RnCN = RnBN*senObject.GetRotations();

	/*North and East Bounds*/	
	gridpose = pose.topRows(2).array() / res;
	minlim = (gridpose.array() - range).max(minsub.array());
	maxlim = (gridpose.array() + range).min(maxsub.array());
	for (int count = 0; count < minlim.rows(); count++) {
		minlim(count) = std::round(minlim(count));
		maxlim(count) = std::round(maxlim(count));
	}
	nrthlen = (size_t)(maxlim(0) - minlim(0) + 1);
	eastlen = (size_t)(maxlim(1) - minlim(1) + 1);
	nrth = RowVectorXd::LinSpaced(nrthlen, minlim(0), maxlim(0));
	east = RowVectorXd::LinSpaced(eastlen, minlim(1), maxlim(1));

	/*Field of View Subscripts*/
	constant = RowVectorXd::Ones(nrthlen);
	subscripts.resize(2, eastlen*nrthlen);
	subscripts.row(0) = nrth.replicate(1,eastlen);
	for (int count = 0; count < eastlen; count++) {
		subscripts.block(1, nrthlen*count, 1, nrthlen) = constant*east(count);
	}
	magnitude = (subscripts.colwise() - gridpose).colwise().hypotNorm();
	index = LogicalIndex(magnitude, magnitude, range, -1);
	cells.resize(2, index.cols());
	for(int count = 0; count < index.cols(); count++){
		cells.col(count) = subscripts.col(index(count)).cast<int>();
	}

	/*Update Grid and Measurement Vectors*/
	Grid(rnCN, RnCN, obsinfo, obscam, cells);
}

/*--------------------------------------------------------------------------------------------------------------------------------------*/

void Mapping::Grid(MatrixXd rnCN, MatrixXd RnCN, MatrixXd obsinfo, VectorXi obscam, MatrixXi cells) {

	/*Memory Allocation*/
	size_t centres = cells.cols();
	RowVectorXi index, filter(centres);
	RowVectorXd norms(centres), magnitude(centres);
	RowVectorXd weight(centres), dtprod(centres), thGC(centres);
	MatrixXd rcPC(3, centres), rcGC(3, centres); 
	Matrix3d RnNC;
	Vector3d rnPN;

	/*Parameter Initialisation*/
	size_t objects = obsinfo.cols();
	MatrixXd rcQC = obsinfo.topRows(3);
	RowVectorXd thQC = obsinfo.row(3);
	RowVectorXd radii = obsinfo.row(4);
	RowVectorXd free = RowVectorXd::Zero(centres);
	MatrixXd rnGN = MatrixXd::Zero(3, centres);
	MatrixXd valid = MatrixXd::Ones(grid.rows(), grid.cols());
	MatrixXd prevgrid = grid;

	/*Updation of Grid*/
	norms = radii.array() / sin(thQC.array());
	rcPC = norms.replicate(3, 1).cwiseProduct(rcQC);
	rnGN.topLeftCorner(cells.rows(), cells.cols()) = (cells.array().cast<double>() + 0.5)*res;
	for (int count = 0; count < objects; count++) {
		rnPN = RnCN.middleCols(obscam(count) * 3, 3)*rcPC.col(count) + rnCN.col(obscam(count));
		rnPN(2) = 0;

		/*Region of Occupancy*/
		magnitude = (rnGN.colwise() - rnPN).colwise().hypotNorm();
		index = LogicalIndex(magnitude, magnitude, radii(count), -1);
		if (index.cols() > 0) {
			for (int loop = 0; loop < index.cols(); loop++) {
				grid(cells(0, index(loop)), cells(1, index(loop))) += inc;
				free(index(loop)) = 1;
			}
		}

		/*Shadow Ellipsoid*/
		RnNC = RnCN.middleCols(obscam(count) * 3, 3).transpose();
		rcGC = (RnNC)*(rnGN.colwise() - rnCN.col(obscam(count)));
		magnitude = rcGC.colwise().hypotNorm();
		weight = (rcGC.cwiseProduct(rcQC.col(count).replicate(1, centres))).colwise().sum();
		dtprod = weight.cwiseQuotient(magnitude);
		index = LogicalIndex(acos(dtprod.array()), dtprod, thQC(count), 0);
		if (index.cols() > 0) {
			for (int loop = 0; loop < index.cols(); loop++) {
				free(index(loop)) = 1;
			}
		}
	}

	/*Decrement Free Cells and Limit Likelihoods*/
	gridy = grid - prevgrid;
	index = LogicalIndex(free, free, 0, 0);
	if (index.cols() > 0) {
		for (int count = 0; count < index.cols(); count++) {
			grid(cells(0, index(count)), cells(1, index(count))) -= dec;
		}
	}
	grid = grid.array().min(valid.array()*maxval);
	grid = grid.array().max(valid.array()*minval); 
}

/*--------------------------------------------------------------------------------------------------------------------------------------*/

RowVectorXi Mapping::LogicalIndex(RowVectorXd uelem, RowVectorXd lelem, const double upper, const double lower) {

	RowVectorXi index;
	RowVectorXi inv = RowVectorXi::Constant(uelem.cols(), -1);
	RowVectorXi lin = RowVectorXi::LinSpaced(uelem.cols(), 0, (const int)(uelem.cols() - 1));
	RowVectorXi filter = ((uelem.array() <= upper) && (lelem.array() >= lower)).select(lin, inv);
	std::sort(filter.data(), filter.data() + filter.cols());
	index = filter.rightCols((filter.array() > -1).count());
	return index;
}

/*--------------------------------------------------------------------------------------------------------------------------------------*/

MatrixXd Mapping::GetGrid(bool flag) {

	MatrixXd output;
	output = (flag) ? gridy : grid;
	return output;
}

/*--------------------------------------------------------------------------------------------------------------------------------------*/

VectorXd Mapping::GetVect(bool flag) {

	VectorXd output;
	output = (flag) ? yhat : y;
	return output;
}






























/*--------------------------------------------------------------------------------------------------------------------------------------*/


	//NOTE:: Camera Index now 0,1,2,3!!! NOT 1,2,3,4!! FIX!!

void Mapping::MeasureLand(MatrixXd rnCN, MatrixXd RnCN, Vector3d rnBN, Matrix3d RnBN, MatrixXd LC, RowVectorXd camL, MatrixXd lmrks) {

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
		MatrixXd ind = 3 * camL.replicate(3, 1) - temp.replicate(1, lnd);
		MatrixXd temp3 = thLC.array().sin();
		MatrixXd temp2 = lmrks.row(3).array().cwiseQuotient(temp3.array());
		MatrixXd rcPC(3, lnd);
		rcPC = temp2.replicate(3, 1).cwiseProduct(rcLC);
		MatrixXd rnPC = MatrixXd::Zero(3, lnd);
		MatrixXd rnPN(3, lnd);

		for (int count = 0; count < lnd; count++) {
			rnPN.col(count) = RnCN.middleCols((camL(count) - 1) * 3, 3)*rcPC.col(count) + rnCN.col(camL(count) - 1);                 //.col(ind.col(count))*rcPC.col(count); // be careful with the use of count. Not sure the 0/1 starting difference issue is resovled in this line
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
		y << rangey, bearingy;
		yhat << rangeyhat, bearingyhat;
	}
	else { // Using a 1 element array with a zero in it to indicate there was no observations
		y.resize(1);
		yhat.resize(1);
		y << 0;
		yhat << 0;
	}
}
