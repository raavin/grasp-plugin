/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <fstream>
#include <iostream>

#include "ForceClosureTest.h"
#include "VectorMath.h"


using namespace cnoid;
using namespace std;
using namespace grasp;


double ForceClosureTest::forceClosureTestEllipsoid(VectorXd &wrench, Vector3* Pc, Vector3* Nc, int points, double mu, double f_max) {
	//thhd1 : thhd1*fmax : radius of ellipsoid (0<thhd1<1)
	//   1 <= thhd2 <= points
	// Pc[]:  the contact position when the object center is zero


	MatrixXd G = MatrixXd::Zero(6, 3 * points);

	Vector3 center(0.0, 0.0, 0.0);
	center = center / (double)points;


	for (int i = 0; i < points; i++) {
		for (int j = 0; j < 3; j++) G(j, j + 3*i) = 1.0;

		Matrix3 tm = sqew(Vector3(Pc[i] - center)); //Confirm ObjCom cordinate

		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++) {
				G(j + 3, k + 3*i) = GAMMA * tm(j, k); ///
			}
	}

	double esize = 0.5;

	Matrix3 Si( diag(1.0 / dbl(mu*f_max) / (1.0 - esize), 1.0 / dbl(mu*f_max) / (1.0 - esize), 1.0 / dbl(f_max) / esize));
	Matrix3 Si2( diag(1.0 / dbl(f_max), 1.0 / dbl(f_max), 1.0 / dbl(f_max / 2.0)));

	double n1 = 1.0;
	double n2 = 0.5;

	MatrixXd U = MatrixXd::Zero(3 * points, 3 * points);
	MatrixXd U2 = MatrixXd::Zero(3 * points, 3 * points);
	VectorXd N(3*points);

	int pnum = 1;
	int bnum = 1;
	int pnum3 = 1;


	for (int i = 0;i < points;i++) {
		pnum *= 2;
	}

	vector <MatrixXd>  Up (pnum);  
	vector <VectorXd>  Np (pnum*2);  

	for (int i = 0;i < pnum;i++) {
		Up[i] = MatrixXd::Zero(3 * points, 3 * points);
		Np[i] = VectorXd(3 * points);
	}

	for (int i = 0; i < points; i++) {

		Vector3 tx(1.0, 0.0, 0.0), ty(0.0, 1.0, 0.0), ni(Nc[i]);

		if (fabs(ni(0)) <= 0.5) {

			Vector3 tmp( cross( cross(ni, tx), ni) );
			tx = tmp / norm2(tmp);

			ty = cross(ni, tx);
			ty = ty / norm2(ty);

		} else {

			Vector3 tmp( cross( cross(ni, ty), ni) );
			ty = tmp / norm2(tmp);

			tx = cross(ty, ni);
			tx = tx / norm2(tx);
		}

		Matrix3 Ui(v3(tx, ty, ni));

		Matrix3 USU(Ui*Si*trans(Ui));
		Matrix3 US2U(Ui*Si2*trans(Ui));

		for (int l = 0;l < pnum;l++) {
			if (l % (bnum*2) < bnum) {
				for (int j = 0; j < 3; j++) {
					for (int k = 0; k < 3; k++) {
						Up[l](3*i + j, 3*i + k) = USU(j, k);
					}
					Np[l](3*i + j) = ni(j) * n1; // normal
				}
			} else {
				for (int j = 0; j < 3; j++) {
					for (int k = 0; k < 3; k++) {
						Up[l](3*i + j, 3*i + k) = US2U(j, k);
					}
					Np[l](3*i + j) = ni(j) * n2;
				}
			}
		}
		bnum *= 2;


	}

	VectorXd phi = G * N;
	MatrixXd UG = inverse(U) * trans(G);
	MatrixXd iGUG = inverse(G * UG);

	int flag = 1;
	double ming = 1000.0;


	vector <bool> iflag(pnum);
	for (int i = 0;i < pnum;i++) iflag[i] = false;

	vector <double>  distance(pnum);

	double dp = (double)points;

	for (int i = 0;i < pnum;i++) {
		phi = G * Np[i];


		UG = inverse(Up[i]) * trans(G);
		MatrixXd GUG(G * UG);
		calcPseudoInverse(GUG, iGUG, 1.0e-10);


		double ans3 = inner_prod( VectorXd(wrench - f_max * phi),  VectorXd( iGUG *( wrench - f_max * phi)) );

		if (ans3 >= dp) {
			double temp = -  min( EvaluateEllipsePointDistance(iGUG, (wrench - f_max * phi), dp), EvaluateEllipsePointDistance2(iGUG, (wrench - f_max * phi), dp));
			distance[i] = temp;
			if (ming < temp || ming > 0) ming = temp;
			flag = -1;
		} else {
			iflag[i] = true;
			double temp =  min( EvaluateEllipsePointDistance(iGUG, (wrench - f_max * phi), dp), EvaluateEllipsePointDistance2(iGUG, (wrench - f_max * phi), dp));
			distance[i] = temp;
			if ( (temp) <= 0) {
				temp = 1000;
			}
			if (temp < ming) {
				ming = temp;
			}
		}

	}
	cout << ming << endl;
	
	if(!isOutputForceSpace && !isOutputTorqueSpace ){
		return ming;
	}

#ifdef VIEW_CONVEX


	vector <MatrixXd> iGUG_(pnum);
	vector <VectorXd> phi_(pnum);

	for (int i = 0;i < pnum;i++) {
		phi_[i] = G * Np[i];
#ifdef VIEW_FORCE_SPACE
		phi_[i][3] = phi_[i][4] = phi_[i][5] = 0;
#else
		phi_[i][0] = phi_[i][1] = phi_[i][2] = 0;  //torque space
#endif

		UG = inverse(Up[i])*trans(G);
		MatrixXd GUG(G* UG);

		for (int j = 0;j < 6;j++) for (int k = 0;k < 6;k++) {
#ifdef VIEW_FORCE_SPACE
				if (j > 2 || k > 2)  GUG(j, k) = (j == k) ? 1 : 0;
#else
				if (j < 3 || k < 3)  GUG(j, k) = (j == k) ? 1 : 0; //torque space
#endif
			}
		calcPseudoInverse(GUG, iGUG, 1.0e-10);
		iGUG_[i] = MatrixXd (iGUG);
	}


	vector<double> exfs, exfs_out;
	VectorXd w(6);
	w[0] =  w[1]  = w[2] = w[3] = w[4] = w[5] = 0;
	for (int j = -100;j < 100;j++) {
		if (j % 10 == 0) cout << j << endl;

		for (int k = -100;k < 100;k++) for (int l = -100;l < 100;l++) {

#ifdef VIEW_FORCE_SPACE
				w[0] = 0.3 * (j);
				w[1] = 0.3 * (k);
				w[2] = 0.3 * (l);
#else
				w[3] = (j); //torque space
				w[4] = (k);
				w[5] = (l);
#endif

				flag = 1;
				for (int i = 0;i < pnum;i++) {
					double ans3 =  inner_prod(w - f_max * phi_[i], iGUG_[i]*(w - f_max * phi_[i]));

					if (ans3 >= dp) {
						flag = -1;
						break;
					}
				}
				if (flag == 1) {
					for (int n = 0; n < 6; n++) exfs.push_back(w(n));
				}
			}
	}
//  calcConvexHull2(6, exfs, exfs_out, wrench, false);
	ConvexAnalysis::outputConvexHull(6, exfs, false);
	cout << "test" << endl;

#endif


}

double ForceClosureTest::NormalForceClosureTest(VectorXd &wrench, Vector3* Pc, Vector3* Nc, int points, double mu, int cface, double f_max) {

	MatrixXd G = MatrixXd::Zero(6, 3 * points);

	for (int i = 0; i < points; i++) {
		for (int j = 0; j < 3; j++)
			G(j, j + 3*i) = 1.0;

//      Matrix3 tm = sqew(Vector3(Pc[i]-objCoM));
		Matrix3 tm = sqew(Vector3(Pc[i])); // Confirm  objCoM

		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				G(j + 3, k + 3*i) = GAMMA * tm(j, k);
	}

	MatrixXd V = MatrixXd::Zero(3 * points, cface * points);

	double mx = mu / ::sqrt(1 + mu * mu), mz = 1 / ::sqrt(1 + mu * mu);

	for (int i = 0; i < points; i++) {

		Vector3 tx(1.0, 0.0, 0.0), ty(0.0, 1.0, 0.0), ni(Nc[i]);

		if (fabs(ni(0)) <= 0.5) {

			Vector3 tmp( cross( cross(ni, tx), ni) );
			tx = tmp / norm2(tmp);

			ty = cross(ni, tx);
			ty = ty / norm2(ty);

		} else {

			Vector3 tmp( cross( cross(ni, ty), ni) );
			ty = tmp / norm2(tmp);

			tx = cross(ty, ni);
			tx = tx / norm2(tx);
		}
		
#define m_pi    (3.141596)

		for (int j = 0; j < cface; j++) {
			Vector3 vi( mz * ni + mx * ( sin(2.0* m_pi * j / (double)cface) * tx + cos(2.0 * m_pi * j / (double)cface) * ty));

			for (int k = 0; k < 3; k++)
				V(3*i + k, cface*i + j) = vi(k);
		}
	}

	MatrixXd GV (G* V);

	//Candidate of 3 points contact
	vector<double> exfs, exfs_out;

	double temp = f_max * sqrt(1.0 + mu * mu);



	if (points > 6) { //For the case of points>6
		int cfacep = 1;
		for (int i = 0;i < points;i++) {
			cfacep *= cface + 1;
		}
		for (int i = 0;i < cfacep;i++) {
			VectorXd lambda = VectorXd(cface * points);
			int p = 1;
			for (int j = 0;j < points;j++) {
				int l = i / p;
				int k = l % (cface + 1);
				if (k != cface)	   lambda(k + j*cface) = temp;
				p *= (cface + 1);
			}
			VectorXd w = (GV* lambda) - wrench;

			for (int n = 0; n < 6; n++)
				exfs.push_back(w(n));
		}
	}

	if (points == 6) { //For the case of points=6
		for (int j = 0; j < cface + 1; j++)
			for (int k = 0; k < cface + 1; k++)
				for (int l = 0; l < cface + 1; l++)
					for (int m = 0; m < cface + 1; m++)
						for (int o = 0; o < cface + 1; o++)
							for (int p = 0; p < cface + 1; p++) {

								VectorXd lambda = VectorXd(cface * points);

								if (j != cface)	   lambda(j)         = temp;
								if (k != cface)	   lambda(k + cface)   = temp;
								if (l != cface)	   lambda(l + 2*cface) = temp;
								if (m != cface)	   lambda(m + 3*cface) = temp;
								if (o != cface)	   lambda(o + 4*cface) = temp;
								if (p != cface)	   lambda(p + 5*cface) = temp;

								VectorXd w = (GV*lambda) - wrench;

								for (int n = 0; n < 6; n++)
									exfs.push_back(w(n));
							}
	}
	if (points == 5) { //For the case of points=5
		for (int j = 0; j < cface + 1; j++)
			for (int k = 0; k < cface + 1; k++)
				for (int l = 0; l < cface + 1; l++)
					for (int m = 0; m < cface + 1; m++)
						for (int o = 0; o < cface + 1; o++) {

							VectorXd lambda = VectorXd(cface * points);

							if (j != cface)	   lambda(j)         = temp;
							if (k != cface)	   lambda(k + cface)   = temp;
							if (l != cface)	   lambda(l + 2*cface) = temp;
							if (m != cface)	   lambda(m + 3*cface) = temp;
							if (o != cface)	   lambda(o + 4*cface) = temp;

							VectorXd w = (GV*lambda) - wrench;

							for (int n = 0; n < 6; n++)
								exfs.push_back(w(n));
						}
	}
	if (points == 4) { //For the case of points=4
		for (int j = 0; j < cface + 1; j++)
			for (int k = 0; k < cface + 1; k++)
				for (int l = 0; l < cface + 1; l++)
					for (int m = 0; m < cface + 1; m++) {

						VectorXd lambda = VectorXd(cface * points);

						if (j != cface)	   lambda(j)         = temp;
						if (k != cface)	   lambda(k + cface)   = temp;
						if (l != cface)	   lambda(l + 2*cface) = temp;
						if (m != cface)	   lambda(m + 3*cface) = temp;

						VectorXd w = (GV*lambda) - wrench;

						for (int n = 0; n < 6; n++)
							exfs.push_back(w(n));
					}
	} else if (points == 3) {
		for (int j = 0; j < cface + 1; j++)
			for (int k = 0; k < cface + 1; k++)
				for (int l = 0; l < cface + 1; l++) {

					VectorXd lambda = VectorXd(cface * points);

					if (j != cface)   lambda(j)         = f_max * sqrt(1.0 + mu * mu);
					if (k != cface)   lambda(k + cface)   = f_max * sqrt(1.0 + mu * mu);
					if (l != cface)   lambda(l + 2*cface) = f_max * sqrt(1.0 + mu * mu);

					VectorXd w = (GV*lambda) - wrench;

					for (int n = 0; n < 6; n++)
						exfs.push_back(w(n));
					//cout << w << endl;
				}
	}
	if (points < 3) {
		cout << " Not supported number of contacts: " << points << endl;
//		if (fingtipGrasp())
//			return -100.0;
//		else
			return -100.0;
	}


	VectorXd w = VectorXd(6);
	for (int m = 0; m < 6; m++)
		exfs.push_back(w(m));

	double ret = 1.0e10;

	//calcConvexHull(6, exfs, exfs_out, false);
	ret = ConvexAnalysis::calcConvexHull2(6, exfs, exfs_out, w, false);////////////////////////////////

#if 0 //3d distance 
	for (int i = 0;i < (int)exfs_out.size() / 18;i++) {
		double a[3][6] ;
		double w2[6];
		for (int j = 0;j < 18;j++) {
			a[j/6][j%6] = exfs_out[i*18+j];
		}
		for (int j = 0;j < 6;j++) {
			w2[j] = wrench[j] - a[0][j];
			a[2][j] -= a[0][j];
			a[1][j] -= a[0][j];
			//a[0][j] = 0;
		}
		double aa = 0, ab = 0, bb = 0, wa = 0, wb = 0;
		for (int j = 0;j < 6;j++) {
			aa += a[1][j] * a[1][j];
			bb += a[2][j] * a[2][j];
			ab += a[1][j] * a[2][j];
			wa += w2[j] * a[1][j];
			wb += w2[j] * a[2][j];
		}
		double adbc = aa * bb - ab * ab;
		double l = (bb * wa - ab * wb) / adbc;
		double m = (-ab * wa + aa * wb) / adbc;

		VectorXd w3 = VectorXd(6);
		for (int j = 0;j < 6;j++) {
			w3[j] = w2[j] - l * a[1][j] - m * a[2][j];
		}
		double tt = norm_2(w3);
		if (tt < ret)  ret = tt;
	}
#endif

// cout << "fc" << ret << endl;

#ifdef VIEW_CONVEX
	if (ret >= 0)
//      displayConvexHull(6, exfs_out, false);
		ConvexAnalysis::outputConvexHull(6, exfs, false);
#endif
	return ret;

}

double ForceClosureTest::EvaluateEllipsePointDistance(MatrixXd ellipse, VectorXd point, double radius) {

	MatrixXd evec = MatrixXd::Zero(6, 6);
	VectorXd eigen(6);
	calcEigenVectors (ellipse, evec, eigen);

	VectorXd normal(6);

	normal = VectorXd ( evec* point);


	//cout <<" a "   << dot(point , normal ) << " " ;

	double a = inner_prod( VectorXd(normal), VectorXd ( ellipse* normal ) );
	double b = inner_prod( VectorXd( point) , VectorXd ( ( ellipse + trans(ellipse)* normal) ) );
	double c = inner_prod(point , VectorXd ( ellipse* point ) ) - radius;

	double D = (b * b - 4.0 * a * c);
	if (D > 0) {
		D = sqrt(D);
		double x1 = (-b + D) / (2.0 * a);
		double x2 = (-b - D) / (2.0 * a);
		double x =  min( fabs(x1) , fabs(x2) );
		return norm_2(normal)*x;
	} else {
		double ans = inner_prod(point, ellipse*point);
		return norm_2(point)*(sqrt(ans) - sqrt(radius)) / sqrt(ans);
	}
}

double ForceClosureTest::EvaluateEllipsePointDistance2(MatrixXd ellipse, VectorXd point, double radius) {

	MatrixXd evec = MatrixXd::Zero(6, 6);
	MatrixXd evec2 = MatrixXd::Zero(6, 6);
	VectorXd eigen(6);
	calcEigenVectors (ellipse, evec2, eigen);

	MatrixXd swap = MatrixXd::Zero(6, 6);
	int eipos[6] = {0, 1, 2, 3, 4, 5};

	for (int i = 0;i < 5;i++) {
		for (int j = 0;j < 5 - i;j++) {
			if (eigen[j] < eigen[j+1]) {
				double temp = eigen[j+1];
				eigen[j+1] = eigen[j];
				eigen[j] = temp;
				int itemp = eipos[j+1];
				eipos[j+1] = eipos[j];
				eipos[j] = itemp;
			}
		}
	}
	for (int i = 0;i < 6;i++) {
		swap(eipos[i], i) = 1.0;
	}
	evec = evec2* swap;

	//cout <<"eigen " <<eigen << endl;

	VectorXd normal(6);

	double xa[4];
	double xz[3];


	double a1, a2, a3, a4, a5, a6;
	a1 = radius / eigen[0];
	a2 = radius / eigen[1];
	a3 = radius / eigen[2];
	a4 = radius / eigen[3];
	a5 = radius / eigen[4];
	a6 = radius / eigen[5];

	VectorXd p2 ( trans(evec) * point);
//	double r2 = dot_(point , prod( ellipse, point) ) ;

	double b1, b2, b3, b4, b5, b6;
	b1 = p2(0) * p2(0);
	b2 = p2(1) * p2(1);
	b3 = p2(2) * p2(2);
	b4 = p2(3) * p2(3);
	b5 = p2(4) * p2(4);
	b6 = p2(5) * p2(5);

//	cout << b1/a1 + b2/a2 +b3/a3 + b4/a4 + b5/a5+ b6/a6 << endl;

	double offset = 1.0 - b4 / a4 - b5 / a5 - b6 / a6;

	xa[3] = offset;
	xa[2] = offset * (a1 + a2 + a3) - (b1 + b2 + b3);
	xa[1] = offset * (a1 * a2 + a2 * a3 + a3 * a1) - (b1 * (a2 + a3) + b2 * (a3 + a1) + b3 * (a1 + a2));
	xa[0] = offset * (a1 * a2 * a3) - (b1 * a2 * a3 + b2 * a3 * a1 + b3 * a1 * a2);

	sol3(xa, xz);
	//cout << " "<< xz[0] << " "<< xz[1] << " "<< xz[2]<< endl;
//	exit(0);

	double maxa = xz[0];
	for (int i = 0;i < 3;i++) {
		if ( (xz[i] != 0) && (fabs(xz[i]) < fabs(maxa)) ) {
			maxa = xz[i];
		}
	}

	MatrixXd eigenrad2 = MatrixXd::Zero(6, 6);
	for (int i = 0;i < 6;i++) {
		eigenrad2(i, i) = 1.0 / (radius / eigen[i] + maxa );
//		eigenrad2(i,i)= 1.0/ (1.0/eigen[i] );
	}
	VectorXd temp ( trans (evec) * point );
	VectorXd temp2 ( eigenrad2 * temp );
	normal = VectorXd ( evec *  temp2);


	//cout << offset << " "<< maxa << " "<<eigenrad2(0,0)*b1 + eigenrad2(1,1)*b2 + eigenrad2(2,2)*b3 + b4/a4 + b5/a5 + b6/a6

	//cout <<" a "   << dot(point , normal ) << " " ;
	//cout <<"er2" << eigenrad2 << endl;
	//cout <<"evec" << evec << endl;

	//cout <<"e2" << prod(evec, MatrixXd ( prod(eigenrad2,trans(evec) ) ) ) << endl;

	double a = inner_prod(normal, (ellipse* normal));
	double b = inner_prod(point, ( (ellipse + trans(ellipse))* normal));
	double c = inner_prod(point , ( ellipse* point) ) - radius;

	double D = (b * b - 4.0 * a * c);
	if (D > 0) {
		D = sqrt(D);
		double x1 = (-b + D) / (2.0 * a);
		double x2 = (-b - D) / (2.0 * a);

		double x =  min( fabs(x1) , fabs(x2) );

		//cout << prod ( trans(evec),point)  <<endl << eigen << endl;
		//cout << evec << endl;
		//cout << VectorXd (normal*x)<<endl ;
		//cout << "point,"<<point << endl;

		//cout <<norm_2(normal)*x << normal*x << endl;
		//exit(0);

		return norm_2(normal)*x;
	} else {
		double ans = inner_prod(point, ellipse* point);
		return norm_2(point)*(sqrt(ans) - sqrt(radius)) / sqrt(ans);
	}

}

double ForceClosureTest::EvaluateEllipsePointDistance3(MatrixXd ellipse, VectorXd point, double radius) {

	MatrixXd evec = MatrixXd::Zero(6, 6);
	MatrixXd evec2 = MatrixXd::Zero(6, 6);
	VectorXd eigen(6);
	calcEigenVectors (ellipse, evec2, eigen);

	MatrixXd swap = MatrixXd::Zero(6, 6);
	int eipos[6] = {0, 1, 2, 3, 4, 5};

	for (int i = 0;i < 5;i++) {
		for (int j = 0;j < 5 - i;j++) {
			if (eigen[j] < eigen[j+1]) {
				double temp = eigen[j+1];
				eigen[j+1] = eigen[j];
				eigen[j] = temp;
				int itemp = eipos[j+1];
				eipos[j+1] = eipos[j];
				eipos[j] = itemp;
			}
		}
	}
	for (int i = 0;i < 6;i++) {
		swap(eipos[i], i) = 1.0;
	}
	evec = evec2 * swap;

	VectorXd normal(6);

	double xa[4];
	double xz[3];


	double a1, a2, a3, a4, a5, a6;
	a1 = radius / eigen[0];
	a2 = radius / eigen[1];
	a3 = radius / eigen[2];
	a4 = radius / eigen[3];
	a5 = radius / eigen[4];
	a6 = radius / eigen[5];

	VectorXd p2 (trans(evec)*point);

	double b1, b2, b3, b4, b5, b6;
	b1 = p2(0) * p2(0);
	b2 = p2(1) * p2(1);
	b3 = p2(2) * p2(2);
	b4 = p2(3) * p2(3);
	b5 = p2(4) * p2(4);
	b6 = p2(5) * p2(5);
	
	
	double offset2=0;
	double maxa=-1.0e10;	
	cout <<"convergence"<< maxa << endl;
	for(int j=0;j<10;j++){

		double offset = 1.0 - b4 / (a4+offset2) - b5 / (a5+offset2) - b6 / (a6+offset2);

		xa[3] = offset;
		xa[2] = offset * (a1 + a2 + a3) - (b1 + b2 + b3);
		xa[1] = offset * (a1 * a2 + a2 * a3 + a3 * a1) - (b1 * (a2 + a3) + b2 * (a3 + a1) + b3 * (a1 + a2));
		xa[0] = offset * (a1 * a2 * a3) - (b1 * a2 * a3 + b2 * a3 * a1 + b3 * a1 * a2);

		sol3(xa, xz);

		maxa = xz[0];
		for (int i = 0;i < 3;i++) {
			if ( (xz[i] != 0) && (fabs(xz[i]) < fabs(maxa)) ) {
				maxa = xz[i];
			}
		}
		offset2 = maxa;
		
		cout <<"convergence"<< maxa << endl;
	}

	MatrixXd eigenrad2 = MatrixXd::Zero(6, 6);
	for (int i = 0;i < 6;i++) {
		eigenrad2(i, i) = 1.0 / (radius / eigen[i] + maxa );
	}
	VectorXd temp ( trans (evec) * point );
	VectorXd temp2 ( eigenrad2* temp  );
	normal = VectorXd ( evec *  temp2);



	double a = inner_prod(normal, (ellipse* normal));
	double b = inner_prod(point, (ellipse + trans(ellipse))* normal);
	double c = inner_prod(point , ( ellipse * point) ) - radius;

	double D = (b * b - 4.0 * a * c);
	if (D > 0) {
		D = sqrt(D);
		double x1 = (-b + D) / (2.0 * a);
		double x2 = (-b - D) / (2.0 * a);

		double x =  min( fabs(x1) , fabs(x2) );

		return norm_2(normal)*x;
	} else {
		double ans = inner_prod(point, (ellipse * point));
		return norm_2(point)*(sqrt(ans) - sqrt(radius)) / sqrt(ans);
	}

}



