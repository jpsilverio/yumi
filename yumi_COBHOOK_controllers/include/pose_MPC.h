#include <stdio.h>
#include <vector>
#include <armadillo>
#include <float.h>

using namespace arma;
mat QuatMatrix(mat q);
mat QuatToRotMat(vec4 q);
mat acoslog(mat x);
mat Qexpfct(mat u);
mat Qlogfct(mat x);
mat Qexpmap(mat u, vec mu);
mat Qlogmap(mat x, vec mu);
mat Qtransp(vec g, vec h);
mat transp(vec g, vec h);
vec logmap(vec x, vec mu);
vec expmap(vec u, vec mu);
vec gaussPDF(mat Data, colvec Mu, mat Sigma);
mat solveAlgebraicRiccati_eig_discrete(mat A, mat B, mat Qx, mat R);
