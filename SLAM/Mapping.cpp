#include "Mapping.h"

Mapping::Mapping(MatrixXd currentgrid, double* params)
{
	/**************************************************************************
	* The Constructor() initialises all parameters specified below for the    *
	* Mapping Class   (CHANGE TO YMAL FILE?)                                  *
	***************************************************************************/

	res = params[0]; dec = params[1];	
	inc = params[2]; thres = params[3];
	maxval = params[4]; minval = params[5];
	grid = currentgrid; gridy = MatrixXd::Zero(grid.rows(), grid.cols());
}

void Mapping::Nav(Vector3d pose, MatrixXd obinfo, MatrixXd landinfo, Sensor senObject) {
	// To DO!
}

void Mapping::Grid(MatrixXd rnCN, MatrixXd RnCN, MatrixXd obinfo, VectorXi camera, MatrixXi cells) {

	/**************************************************************************
	 * The Grid Function() updates a virtual grid which retains all history   *
	 * of the environment, as well as gridy, which contains only the current  *
	 * obstacle information from the cameras.                                 *
	 **************************************************************************/

	size_t centres = cells.cols();
	MatrixXd rcGC(3, centres), rnGN(3, centres);
	MatrixXd rcQCmat(3, centres), rnPNmat(3, centres), rnCNmat(3, centres);
	RowVectorXd magnitude(centres), weight(centres), dtprod(centres), thGC(centres);
	Matrix3d trans;

	/*Parameter Initialisation*/

	size_t objects = obinfo.cols();
	MatrixXd rcQC = obinfo.topRows(3);
	RowVectorXd thQC = obinfo.row(3);
	RowVectorXd radii = obinfo.row(4);

	/*Derivation of Global Postions of Detected Obstacles*/

	MatrixXd rnPN(3, objects);
	RowVectorXd norms = radii.array() / sin(thQC.array());
	MatrixXd rcPC = norms.replicate(3, 1).cwiseProduct(rcQC);
	for (int count = 0; count < objects; count++) {
		rnPN.col(count) = RnCN.middleCols((camera(count) - 1) * 3, 3)*rcPC.col(count) + rnCN.col(camera(count) - 1);
	}
	rnPN.bottomRows(1) *= 0;

	/*Updation of Grid*/

	MatrixXd prevgrid = grid;
	RowVectorXd free = RowVectorXd::Ones(centres);
	rnGN.topLeftCorner(cells.rows(), cells.cols()) = cells.array().cast<double>() - 0.5;
	for (int count = 0; count < objects; count++) {
		rnPNmat = rnPN.col(count).replicate(1, centres);
		rcQCmat = rcQC.col(count).replicate(1, centres);
		rnCNmat = rnCN.col(camera(count) - 1).replicate(1, centres);

		/*Region of Occupancy*/

		magnitude = (rnPNmat - rnGN).colwise().norm();
		RowVectorXi index = LogicalIndex(magnitude, radii(count), 1);
		if (index.cols() > 0) {
			for (int loop = 0; loop < index.size(); loop++) {
				grid(cells(0, index(loop)), cells(1, index(loop))) += inc;
				free(index(loop)) = 0;
			}
		}
		/*Shadow Ellipsoid*/

		trans = RnCN.middleCols((camera(count) - 1) * 3, 3).transpose();
		rcGC = trans*(rnGN - rnCNmat);
		magnitude = rcGC.colwise().norm();
		weight = (rcQCmat.cwiseProduct(rcGC)).colwise().sum();
		dtprod = weight.cwiseQuotient(magnitude);
		RowVectorXi indangles = LogicalIndex(acos(dtprod.array()), thQC(count), 0);
		RowVectorXi inddtprod = LogicalIndex(dtprod, 0, 1);
		index = IsMember(indangles, inddtprod);
		if (index.cols() > 0) {
			for (int loop = 0; loop < index.size(); loop++) {
				free(index(loop)) = 0;
			}
		}
	}
	/*Decrement FoV Free Cells and Limit Likelihood Ranges*/

	RowVectorXi index = LogicalIndex(free, 1, 1);
	if (index.cols() > 0) {
		for (int count = 0; count < index.size(); count++) {
			grid(cells(0, index(count)), cells(1, index(count))) -= dec;
		}
	}
	MatrixXd ones = MatrixXd::Ones(grid.rows(), grid.cols());
	gridy = grid - prevgrid;	
	grid = grid.array().min(ones.array()*maxval);
	grid = grid.array().max(ones.array()*minval);
}

RowVectorXi Mapping::IsMember(RowVectorXi set, RowVectorXi subset) {

	/************************************************************************************
	* The IsMember Function() returns all elements of Subset who also belong within Set *
	*************************************************************************************/

	RowVectorXi members, index;
	if (set.size()*subset.size() > 0) {
		members = RowVectorXi::Zero(subset.size());
		for (int count = 0; count < subset.size(); count++) {
			index = (set.array() == subset(count)).select(RowVectorXi::Ones(set.size()), RowVectorXi::Zero(set.size()));
			members(count) = (index.prod()) ? subset(count) : 0;
		}
		std::sort(index.data(), index.data() + index.size());
		RowVectorXi members = index.rightCols((index.array() > 0).count());
	}
	return members;
}

RowVectorXi Mapping::LogicalIndex(RowVectorXd elem, const double cond, bool flag) {

	/**********************************************************************************
	* The LogicalIndex Function() places the index of all elements of a RowVectorXd	  *
	* which satisfies a specific logical condition, into a RowVectorXd named index.   *
	* Boolean Flag switches the equality sign.       								  *
	***********************************************************************************/

	RowVectorXi filter(elem.size());
	RowVectorXi linear = RowVectorXi::LinSpaced(elem.size(), 1, (const int)elem.size());
	if (flag) {
		filter = (elem.array() >= cond).select(linear, RowVectorXi::Zero(elem.cols()));
	} else {
		filter = (elem.array() <= cond).select(linear, RowVectorXi::Zero(elem.cols()));
	}
	std::sort(filter.data(), filter.data() + filter.size());
	RowVectorXi index = filter.rightCols((filter.array() > 0).count());
	return index.array() - 1;
}

MatrixXd Mapping::GetGrid(bool flag) {

	/**********************************************************************************
	* The GetGrid Function() outputs a specified grid dependant upon flag value       *
	***********************************************************************************/

	MatrixXd output;
	output = (flag) ? gridy : grid;
	return output;
}