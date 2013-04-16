/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _ARM_H
#define _ARM_H

#include <stdlib.h>
#include <time.h>
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  

#include "exportdef.h"

namespace grasp{

class Arm;
typedef Arm* ArmPtr;
	
class Arm;
typedef Arm* ArmPtr;


class Arm{
	public:
		Arm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm);
		virtual ~Arm(){};
		cnoid::JointPathPtr arm_path;
		cnoid::Link *palm;
		int nJoints;
			
		std::vector<double> armStandardPose;
		cnoid::Vector3 base_p;
		cnoid::Matrix3 base_R;
			
		virtual bool IK_arm(const cnoid::Vector3 &p, const cnoid::Matrix3 &R);
		virtual bool IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R, const cnoid::VectorXd& q_old);
		virtual bool IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R, double phi, const cnoid::VectorXd& q_old=cnoid::VectorXd::Zero(7));
		virtual bool getPalmPos(const cnoid::Vector3& Pco1, const cnoid::Vector3& Pco2, const cnoid::Matrix3& Rp, const cnoid::Vector3& pPcr1, const cnoid::Matrix3& pRcr1, cnoid::Vector3& Pp, cnoid::VectorXd& theta);


//		bool IK_arm2(const Vector3 &p, const Matrix3 &R);
		virtual double IndexFunc(double a, double b);
		virtual cnoid::VectorXd calcGradient(double a, double b);
		virtual bool checkArmLimit();
		virtual double Manipulability();			
		virtual double avoidAngleLimit();
		virtual double avoidAngleLimit2();
		virtual void calcJacobian(cnoid::MatrixXd& J);
			
		virtual bool closeArm(int lk, int iter, cnoid::Vector3 &oPos, cnoid::Vector3 &objN);
		cnoid::ColdetLinkPairPtr palmObjPair;
		bool palmContact;
		cnoid::Vector3 closeDir;
		cnoid::Vector3 approachOffset;
		
		bool searchBasePositionMode;
};

}

#endif
