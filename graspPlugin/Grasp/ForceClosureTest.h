/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _ForceClosureTest_H
#define _ForceClosureTest_H

#include <cnoid/EigenTypes>

#include "ConvexAnalysis.h"

#include <algorithm>
#include <time.h>
#include <sys/resource.h>

#include "exportdef.h"


namespace grasp{

double getrusage_sec();
double _drand();
void _initrand();
		
class ForceClosureTest
{
	public:
		ForceClosureTest() {
			 GAMMA = 1000.0;
			 isOutputForceSpace=false;
			 isOutputTorqueSpace=false;
		}
		~ForceClosureTest(){}
			
		void SetGAMMA(double GAMMA){
			this->GAMMA= GAMMA;
		}

	static ForceClosureTest* instance(ForceClosureTest *fct=NULL) {
		static ForceClosureTest* instance = (fct) ? fct : new ForceClosureTest();
		return instance;
	}
			

	double forceClosureTestEllipsoid(cnoid::VectorXd &wrench, cnoid::Vector3 Pc[], cnoid::Vector3 Nc[], int points, double mu, double f_max);
	double forceClosureTestEllipsoidSoftFinger(cnoid::VectorXd &wrench, cnoid::Vector3 Pc[], cnoid::Vector3 Nc[], int points, double mu, double f_max, const std::vector<double>& en);
	double forceClosureTestEllipsoidInternal(cnoid::VectorXd &wrench, cnoid::Vector3 Pc[], cnoid::Vector3 Nc[], int points, double mu, double f_max);
	double NormalForceClosureTest(cnoid::VectorXd& wrench, cnoid::Vector3 cpos[], cnoid::Vector3 Nc[], int points, double mu, int cface, double f_max);
	//bool NormalFormClosureTest(cnoid::Vector3* Pc, cnoid::Vector3* Nc, int points, std::vector<double>& spanVectors);
	double ForceClosureTestManipulationForce(cnoid::VectorXd &wrench, cnoid::Vector3 Pc[], cnoid::Vector3 Nc[], int points, double mu, double gamma,cnoid::MatrixXd jacobi[]);
	
	void forceClosureTestOnly();
	
	protected: 


	double EvaluateEllipsePointDistance(cnoid::MatrixXd ellipse, cnoid::VectorXd point, double radius);
	double EvaluateEllipsePointDistance2(cnoid::MatrixXd ellipse, cnoid::VectorXd point, double radius);
	double EvaluateEllipsePointDistance3(cnoid::MatrixXd ellipse, cnoid::VectorXd point, double radius);
	
	// GAMMA is scale value for moment
	double GAMMA;
	bool isOutputForceSpace;
	bool isOutputTorqueSpace;

			
};

}

#endif
