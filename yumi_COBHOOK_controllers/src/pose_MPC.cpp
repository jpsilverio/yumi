/*
 * Taken from demo_Riemannian_pose_infHorLQR.cpp
 * Author: Andras Kupcsik
 * Modified by: João Silvério
 * 2019
 */

#include <pose_MPC.h>

using namespace arma;


//-----------------------------------------------

arma::mat QuatMatrix(arma::mat q) {
	arma::mat Q;
	Q = { { q(0),-q(1),-q(2),-q(3)},
		{	q(1), q(0),-q(3), q(2)},
		{	q(2), q(3), q(0),-q(1)},
		{	q(3),-q(2), q(1), q(0)}};

	return Q;
}

//-----------------------------------------------

mat QuatToRotMat(vec4 q) {
	float w = q(0);
	float x = q(1);
	float y = q(2);
	float z = q(3);
	mat RotMat(3, 3);
	RotMat << 1 - 2 * y * y - 2 * z * z << 2 * x * y - 2 * z * w << 2 * x * z + 2 * y * w << endr << 2 * x * y + 2 * z * w << 1 - 2 * x * x - 2 * z * z
			<< 2 * y * z - 2 * x * w << endr << 2 * x * z - 2 * y * w << 2 * y * z + 2 * x * w << 1 - 2 * x * x - 2 * y * y << endr;
	return RotMat;
}

//-----------------------------------------------

arma::mat acoslog(arma::mat x) {
	arma::mat acosx(1, x.size());

	for (int n = 0; n <= x.size() - 1; n++) {
		if (x(0, n) >= 1.0)
			x(0, n) = 1.0;
		if (x(0, n) <= -1.0)
			x(0, n) = -1.0;
		if (x(0, n) < 0) {
			acosx(0, n) = acos(x(0, n)) - M_PI;
		} else
			acosx(0, n) = acos(x(0, n));
	}

	return acosx;
}

//-----------------------------------------------

arma::mat Qexpfct(arma::mat u) {
	arma::mat normv = sqrt(pow(u.row(0), 2) + pow(u.row(1), 2) + pow(u.row(2), 2));
	arma::mat Exp(4, u.n_cols);

	Exp.row(0) = cos(normv);
	Exp.row(1) = u.row(0) % sin(normv) / normv;
	Exp.row(2) = u.row(1) % sin(normv) / normv;
	Exp.row(3) = u.row(2) % sin(normv) / normv;

	return Exp;
}

//-----------------------------------------------

arma::mat Qlogfct(arma::mat x) {
	arma::mat fullone;
	fullone.ones(size(x.row(0)));
	arma::mat scale(1, x.size() / 4);
	scale = acoslog(x.row(0)) / sqrt(fullone - pow(x.row(0), 2));

	if (scale.has_nan()) {
		scale = 1.0;
	}

	arma::mat Log(3, x.size() / 4);
	Log.row(0) = x.row(1) % scale;
	Log.row(1) = x.row(2) % scale;
	Log.row(2) = x.row(3) % scale;

	return Log;
}

//-----------------------------------------------

arma::mat Qexpmap(arma::mat u, arma::vec mu) {
	arma::mat x = QuatMatrix(mu) * Qexpfct(u);
	return x;
}

//-----------------------------------------------

arma::mat Qlogmap(arma::mat x, arma::vec mu) {
	arma::mat pole;
	arma::mat Q(4, 4, fill::ones);

	pole = {1,0,0,0};

	if (norm(mu - trans(pole)) < 1E-6)
		Q = { { 1,0,0,0},
			{	0,1,0,0},
			{	0,0,1,0},
			{	0,0,0,1}};
		else
		Q = QuatMatrix(mu);

	arma::mat u;
	u = Qlogfct(trans(Q) * x);

	return u;
}

//-----------------------------------------------

arma::mat Qtransp(vec g, vec h) {
	mat E;
	E << 0.0 << 0.0 << 0.0 << endr << 1.0 << 0.0 << 0.0 << endr << 0.0 << 1.0 << 0.0 << endr << 0.0 << 0.0 << 1.0;
	colvec tmpVec = zeros(4, 1);
	tmpVec.subvec(0, 2) = Qlogmap(h, g);

	vec vm = QuatMatrix(g) * tmpVec;
	double mn = norm(vm, 2);
	mat Ac;
	if (mn < 1E-10) {
		Ac = eye(3, 3);
	}

	colvec uv = vm / mn;

	mat Rpar = eye(4, 4) - sin(mn) * (g * uv.t()) - (1 - cos(mn)) * (uv * uv.t());

	Ac = E.t() * QuatMatrix(h).t() * Rpar * QuatMatrix(g) * E;

	return Ac;
}

//-----------------------------------------------

arma::mat transp(vec g, vec h) {
	mat Ac = eye(6, 6);
	Ac.submat(3, 3, 5, 5) = Qtransp(g.subvec(3, 6), h.subvec(3, 6));
	return Ac;
}

//-----------------------------------------------

arma::vec logmap(vec x, vec mu) {
	vec u = join_vert(x.subvec(0, 2), Qlogmap(x.subvec(3, 6), mu.subvec(3, 6)));
	return u;
}

//-----------------------------------------------

arma::vec expmap(vec u, vec mu) {
	vec x = join_vert(u.subvec(0, 2), Qexpmap(u.subvec(3, 5), mu.subvec(3, 6)));
	return x;
}

//-----------------------------------------------

arma::vec gaussPDF(mat Data, colvec Mu, mat Sigma) {

	int nbVar = Data.n_rows;
	int nbData = Data.n_cols;
	Data = Data.t() - repmat(Mu.t(), nbData, 1);

	vec prob = sum((Data * inv(Sigma)) % Data, 1);

	prob = exp(-0.5 * prob) / sqrt(pow((2 * datum::pi), nbVar) * det(Sigma) + DBL_MIN);

	return prob;
}

//-----------------------------------------------

arma::mat solveAlgebraicRiccati_eig_discrete(mat A, mat B, mat Qx, mat R) {
	// Ajay Tanwani, 2016

	int n = A.n_rows;
	mat Q = (Qx + Qx.t()) / 2; // mat Q here corresponds to (Q+Q')/2 specified in set_problem
	mat G = B * R.i() * B.t();
	mat Z(2 * n, 2 * n);
	Z(span(0, n - 1), span(0, n - 1)) = A + G * inv(A.t()) * Q;
	Z(span(0, n - 1), span(n, 2 * n - 1)) = -G * inv(A.t());
	Z(span(n, 2 * n - 1), span(0, n - 1)) = -inv(A.t()) * Q;
	Z(span(n, 2 * n - 1), span(n, 2 * n - 1)) = inv(A.t());

	//Using diagonalization
	cx_mat V(2 * n, 2 * n), U(2 * n, n);
	cx_vec dd(2 * n);

	arma::eig_gen(dd, V, Z);

	int i = 0;
	for (int j = 0; j < 2 * n; j++) {
		if (norm(dd(j)) < 1) {
			U.col(i) = V.col(j);
			i++;
		}
	}

	mat X = real(U(span(n, 2 * n - 1), span(0, n - 1)) * U(span(0, n - 1), span(0, n - 1)).i());

	return X;
}

