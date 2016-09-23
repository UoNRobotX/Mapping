#include "Mapping.h"

/*--------------------------------------------------------------------------------------------------------------------------------------*/

Mapping::Mapping(MatrixXd currentgrid, MatrixXd markers, double* params) {

	landmarks = markers;
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
	double gridrange = senObject.GetRange() / res;
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
	minlim = (gridpose.array() - gridrange).max(minsub.array());
	maxlim = (gridpose.array() + gridrange).min(maxsub.array());
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
	index = LogicalIndex(magnitude, magnitude, gridrange, -1);
	cells.resize(2, index.cols());
	for(int count = 0; count < index.cols(); count++){
		cells.col(count) = subscripts.col(index(count)).cast<int>();
	}

	/*Update Grid and Measurement Vectors*/
	Grid(rnCN, RnCN, obsinfo, obscam, cells);
	ObstVects(rnBN.topRows(2),cells, senObject.GetRange());
	LandVects(rnCN, RnCN, rnBN, RnBN, lndinfo, lndcam);
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

	/*Free Cells*/
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

void Mapping::ObstVects(Vector2d rnBN, MatrixXi cells, double range) {
	
	/*Memory Allocation*/
	size_t index, ang;
	RowVectorXd magnitude, logic;
	MatrixXd measure = MatrixXd::Constant(360, 2, range);	
	MatrixXd threslim, occupied, vectors, current(grid.rows(), grid.cols());
	double threshold, radian;
	for (int count = 0; count < 2; count++) {

		/*Occupied Cell Centroid Positions*/
		index = 0;
		threslim = MatrixXd::Constant(2, cells.cols(), -1);
		current = (count) ? grid : gridy;
		threshold = (count) ? thres : inc;	
		for (int loop = 0; loop < cells.cols(); loop++) {
			if (current(cells(0, loop), cells(1, loop)) >= threshold) {
				threslim.col(index) = cells.col(loop).cast<double>();
				++index;
			}
		}
		logic = threslim.row(0);
		occupied = (threslim.leftCols((logic.array() > -1).count()).array() + 0.5)*res;

		/*Measured Angular Ranges around Vehicle*/
		if (occupied.cols() > 0) {
			vectors = occupied.colwise() - rnBN;
			magnitude = vectors.colwise().hypotNorm();
			for (int loop = 0; loop < vectors.cols(); loop++) {
				radian = atan2(vectors(1, loop), vectors(0, loop));
				radian = (radian <= 0) ? radian + 2 * M_PI : radian;
				radian = (radian > 2 * M_PI) ? radian - 2 * M_PI : radian;
				ang = (size_t)(std::round(radian*(180 / M_PI))-1);
				ang = (ang >= 360) ? 359 : ang;
				ang = (ang < 0) ? 0 : ang;
				measure(ang, count) = (magnitude(loop) < measure(ang, count)) ? magnitude(loop) : measure(ang, count);
			}
		}
	}

	/*Place Ranges into Vectors*/
	yobs = measure.col(0);
	yobshat = measure.col(1);
}

/*--------------------------------------------------------------------------------------------------------------------------------------*/

void Mapping::LandVects(MatrixXd rnCN, MatrixXd RnCN, Vector3d rnBN, Matrix3d RnBN, MatrixXd lndinfo, VectorXi lndcam) {

	/*Memory Allocation*/
	ylnd = VectorXd::Zero(1);
	ylndhat = VectorXd::Zero(1);
	if (lndinfo.rows() > 0) {  		
		double centroid;
		size_t markers = lndinfo.cols();
		VectorXd rangey(markers), rangeyhat(markers);
		VectorXd bearingy(markers), bearingyhat(markers);
		MatrixXd rcPC(3, markers), diff(3, markers);
		MatrixXd truevect(3, markers), landpose;
		Vector3d rnPN;
		
		/*Parameter Initialisation*/
		MatrixXd rcLC = lndinfo.topRows(3);
		RowVectorXd thLC = lndinfo.row(3);
		RowVectorXd id = lndinfo.row(4);

		/*Determine Range and Bearing from Camera Measurements*/
		for (int count = 0; count < markers; count++) {
			centroid = landmarks(3, (size_t)id(count));
			rcPC = rcLC.col(count)*(centroid/sin(thLC(count)));
			rnPN = RnCN.middleCols(lndcam(count) * 3, 3)*rcPC + rnCN.col(lndcam(count));
			diff.col(count) = rnPN - rnBN;
			bearingy(count) = atan2(diff(1, count), diff(0, count));
		}
		bearingy = (bearingy.array() < 0).select(bearingy.array() + 2*M_PI, bearingy);
		bearingy = (bearingy.array() >= 2*M_PI).select(bearingy.array() - 2 * M_PI, bearingy);
		rangey = diff.colwise().hypotNorm();

		/*Assumed True Measurements from NED Landmark Locations*/
		landpose = landmarks.topRows(3);
		for (int count = 0; count < markers; count++) {
			truevect.col(count) = landpose.col((size_t)id(count)) - rnBN;
			bearingyhat(count) = atan2(truevect(1, count), truevect(0, count));
		}
		bearingyhat = (bearingyhat.array() < 0).select(bearingyhat.array() + 2 * M_PI, bearingyhat);
		bearingyhat = (bearingyhat.array() >= 2 * M_PI).select(bearingyhat.array() - 2 * M_PI, bearingyhat);
		rangeyhat = truevect.colwise().hypotNorm();

		/*Form Outputs*/
		ylnd.resize(2 * markers); ylndhat.resize(2 * markers);
		ylnd.topRows(markers) = rangey;
		ylnd.bottomRows(markers) = bearingy;
		ylndhat.topRows(markers) = rangeyhat;
		ylndhat.bottomRows(markers) = bearingyhat;
	}
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
	if (flag) {
		output.resize(yobshat.rows() + ylndhat.rows());
		output.topRows(yobshat.rows()) = yobshat;
		output.bottomRows(ylndhat.rows()) = ylndhat;		
	}else {
		output.resize(yobs.rows() + ylnd.rows());
		output.topRows(yobs.rows()) = yobs;
		output.bottomRows(ylnd.rows()) = ylnd;
	}
	return output;
}

/*--------------------------------------------------------------------------------------------------------------------------------------*/