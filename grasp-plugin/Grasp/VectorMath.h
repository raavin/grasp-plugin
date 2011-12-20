// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _VectorMath_H
#define _VectorMath_H

#include <vector>
#include <cnoid/EigenTypes>
#include <Eigen/Eigenvalues> 
#include "exportdef.h"

namespace grasp{

	typedef cnoid::Matrix3 Matrix33;
	typedef cnoid::VectorXd dvector ;
	typedef cnoid::MatrixXd dmatrix ;
	
	cnoid::Matrix3 sqew(const cnoid::Vector3& a);
	cnoid::Matrix3 diag(double a, double b, double c);
	cnoid::Matrix3 v3(const cnoid::Vector3& x, const cnoid::Vector3& y, const cnoid::Vector3& z);
	cnoid::Matrix3 m33(const cnoid::Vector3& x);
	cnoid::Matrix3 d2v(const cnoid::MatrixXd& a);
	cnoid::MatrixXd v2d(const cnoid::Matrix3& a);
	cnoid::Vector3 col(const cnoid::Matrix3 &a, int n);
	cnoid::Vector3 sort(const cnoid::Vector3 &a);
	cnoid::Vector3 average(const std::vector<cnoid::Vector3> &a);

	double dbl(double a);
	double dbl(cnoid::Vector3 &a);

	double vmax(const std::vector<double> &a);
	int argmax(const std::vector<double> &a);
	double vmin(const std::vector<double> &a);
	int argmin(const std::vector<double> &a);
	double abs(const cnoid::Vector3& a);
	cnoid::Vector3 unit(const cnoid::Vector3& a);

	double det33(const cnoid::Matrix3& V);
	cnoid::Vector3 cross( cnoid::Vector3 v1,cnoid::Vector3 v2);
	double norm2( cnoid::Vector3 v);
	double norm_2( cnoid::VectorXd v);
	cnoid::MatrixXd inverse( cnoid::MatrixXd m);
	cnoid::Matrix3 trans(const cnoid::Matrix3& m);
	cnoid::MatrixXd trans(const cnoid::MatrixXd& m);
	double min ( cnoid::Vector3 v );
	double min ( double a, double b );
	double dot( cnoid::Vector3 v1,cnoid::Vector3 v2);
	double inner_prod( cnoid::VectorXd v1,cnoid::VectorXd v2);
	int calcEigenVectors(const cnoid::MatrixXd &_a, cnoid::MatrixXd  &_evec, cnoid::VectorXd &_eval);
	int calcPseudoInverse(const cnoid::MatrixXd &_a, cnoid::MatrixXd &_a_pseu, double _sv_ratio=1.0e-3);
	int solveLinearEquation(const cnoid::MatrixXd &_a, const cnoid::VectorXd &_b, cnoid::VectorXd &_x, double _sv_ratio=1.0e-3);
	cnoid::Vector3 omegaFromRot(const cnoid::Matrix3& r);
	cnoid::VectorXd prod (const cnoid::MatrixXd &_a, const cnoid::VectorXd &_b);
	cnoid::MatrixXd prod (const cnoid::MatrixXd &_a, const cnoid::MatrixXd &_b);
	
	template<class V> inline void setVector3(const cnoid::Vector3& v3, V& v, size_t top = 0){
		v[top++] = v3(0); v[top++] = v3(1); v[top] = v3(2);
	}
	
	template<class V> inline void setVector3(const cnoid::Vector3& v3, const V& v, size_t top = 0){
		v[top++] = v3(0); v[top++] = v3(1); v[top] = v3(2);
	}
	
	template<class V> inline void getVector3(cnoid::Vector3& v3, const V& v, size_t top = 0){
		v3(0) = v[top++]; v3(1) = v[top++]; v3(2) = v[top]; 
	}
	
	template<class M> inline void setVector3(const cnoid::Vector3& v3, M& m, size_t row, size_t col){
		m(row++, col) = v3(0); m(row++, col) = v3(1); m(row, col) = v3(2); 
	}
    
	template<class M> inline void getVector3(cnoid::Vector3& v3, const M& m, size_t row, size_t col){
		v3(0) = m(row++, col);
		v3(1) = m(row++, col);
		v3(2) = m(row, col);
	}
	
	void calcRodrigues(cnoid::Matrix3& out_R, const cnoid::Vector3& axis, double q);
	void calcRotFromRpy(cnoid::Matrix3& out_R, double r, double p, double y);
	cnoid::Vector3 rpyFromRot(const cnoid::Matrix3& m);
	cnoid::Matrix3 rodrigues(const cnoid::Vector3& axis, double q);
	cnoid::Matrix3 rotFromRpy(const cnoid::Vector3& rpy);
	cnoid::Matrix3 rotFromRpy(double r, double p, double y);
	double det(const cnoid::MatrixXd &_a);
	
	double distance(const cnoid::Vector3 & pt, const cnoid::VectorXd &a);
	cnoid::Vector3 normalPoint(const cnoid::Vector3& pt, const cnoid::VectorXd& r);
	cnoid::Vector3 normalPoint(const cnoid::Vector3& pt, const cnoid::Vector3& p, const cnoid::Vector3& e);
	cnoid::Vector3 projection(const cnoid::Vector3& e, const cnoid::VectorXd& r);
	
	double cos_th(const cnoid::Vector3& p0, const cnoid::Vector3& p1, const cnoid::Vector3& p2);
	void parallelPlane(const cnoid::VectorXd& r0, double l, cnoid::VectorXd& r1);
	bool calcCommonLine(const cnoid::VectorXd& r0, const cnoid::VectorXd& r1, cnoid::Vector3& p, cnoid::Vector3& e);
	double area(const cnoid::Vector3& p0, const cnoid::Vector3& p1, const cnoid::Vector3& p2);
	cnoid::Vector3 calcCOM(const std::vector<cnoid::Vector3>& p);
	void sort_by(std::vector<cnoid::Vector3>& p, std::vector<double>& dist);
	void sort_by(std::vector<int>& p, std::vector<double>& dist);
	bool included(int a, const std::vector<int> b);
	int argument(const std::vector<double>& a, double b);
	int argument(const std::vector<int>& a, int n);
	double minDistancePoints(const cnoid::Vector3& p1, const cnoid::Vector3& nn1, const cnoid::Vector3& p2, const cnoid::Vector3& nn2, cnoid::Vector3& q1, cnoid::Vector3& q2); //minimum distance points between two lines
	cnoid::Vector3 intersectionPoint(const cnoid::Vector3& p1, const cnoid::Vector3& nn1, const cnoid::Vector3& planePt, const cnoid::Vector3& planeN);//intersection between line and plane
	
	double cpoly4(double xa[4], double z);
	/// second degree equation 
	int sol2(double xa[3], double xz[2]);
	/// third degree equation 
	int sol3(double xa[4], double xz[3]);
	/// fourth degree equation 
	int sol4(double xa[5], double xz[4]);
	
	class Homogenous
	{
	public:
		cnoid::Vector3 p;
		cnoid::Matrix3 R;
	};
}

#endif
