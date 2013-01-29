// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#ifndef MANIPCONTROLLER_H
#define MANIPCONTROLLER_H

#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string>
#include <fstream>
#include <iostream>

#include <algorithm>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/resource.h>

#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
#include <cnoid/MessageView>
#include <cnoid/Body>
#include <cnoid/ModelNodeSet>
#include <cnoid/Link>
#include <cnoid/ColdetLinkPair>
#include <cnoid/JointPath>
#include <boost/filesystem.hpp>

#include "../Grasp/VectorMath.h"
#include "../Grasp/GraspController.h"
#include "../Grasp/readtext.h"
#include "../Grasp/Arm.h"
#include "../Grasp/PlanBase.h"
#include "ClusterParameter.h"
#include "PlacePlanner.h"

namespace grasp{
namespace GripperManipulation{

class ManipController : public grasp::GraspController
{

public :
		static ManipController* instance();
		ManipController();
		~ManipController();

		virtual bool initial(TargetObject* targetObject, ArmFingers* targetArmFinger);
		virtual bool doGraspPlanning();
		bool searchPickAndPlaceMotion(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& Pp_put, cnoid::Matrix3& Rp_put, cnoid::VectorXd &theta_, bool edge_grasp);
		bool searchPickAndPlaceWithRegrasp(cnoid::Vector3& Pp_grasp2, cnoid::Matrix3& Rp_grasp2, cnoid::Vector3& Pp_tmp_put, cnoid::Matrix3& Rp_tmp_put, cnoid::Vector3& Pp_regrasp, cnoid::Matrix3& Rp_regrasp, cnoid::Vector3& Pp_put2, cnoid::Matrix3& Rp_put2, cnoid::VectorXd& th_grasp2, cnoid::VectorXd& th_put2, bool edge_grasp);

		bool searchSurfaceSurface(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::VectorXd &theta_, bool edge_grasp){
				cnoid::Vector3 Pp_put = cnoid::Vector3::Zero();
				cnoid::Matrix3 Rp_put = cnoid::Matrix3::Identity();
				return searchPickAndPlaceMotion(Pp_grasp, Rp_grasp, Pp_put, Rp_put, theta_, edge_grasp);
		};

		virtual void calcJointSeq(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& Pp_put, cnoid::Matrix3& Rp_put);
		virtual bool calcJointSeqTest(cnoid::Vector3& Pp_grasp, cnoid::Matrix3& Rp_grasp, cnoid::Vector3& Pp_put, cnoid::Matrix3& Rp_put, cnoid::Vector3& appVec);
		virtual bool calcJointSeqTestRegrasp(cnoid::Vector3& P0, cnoid::Matrix3& R0, cnoid::Vector3& P1, cnoid::Matrix3& R1, cnoid::Vector3& P2, cnoid::Matrix3& R2, cnoid::Vector3& P3, cnoid::Matrix3& R3, cnoid::Vector3& appVec, cnoid::Vector3& appVec2);
		void chooseArmToGrasp();
		void chooseArmToPut();
		void selectManipStrategy();

		bool initPalm;
		int intention, strategy;
		std::vector<cnoid::Vector3> Po_put, Po_tmp; //Object putting position
		std::vector<cnoid::Matrix3> Ro_put, Ro_tmp;
		enum ManipStrategies { RIGHT_RIGHT, LEFT_LEFT, RIGHT_LEFT, LEFT_RIGHT, RIGHT_PUT_RIGHT, LEFT_PUT_LEFT, RIGHT_PUT_LEFT, LEFT_PUT_RIGHT};
		enum Hand { RIGHT, LEFT };
		enum Robots {PA10, HIRO, HRP2, OTHER};
		int graspingHand, robot;

		std::ostream& os;

protected :
		void calcArmJointSeq(cnoid::VectorXd& armJointSeq, ArmPtr& arm_);
		bool setMotionSeq(int graspingState, int contactState, const cnoid::Vector3& P, const cnoid::Matrix3& R, cnoid::VectorXd& jointSeq, ArmPtr& arm_, FingerPtr fingers_[], double time);
		bool setMotionSeqDual(int graspingState, int graspingState2, int contactState, const cnoid::Vector3& P, const cnoid::Matrix3& R, const cnoid::Vector3& P2, const cnoid::Matrix3& R2, cnoid::VectorXd& jointSeq, ArmPtr& arm_, FingerPtr fingers_[], double time);
		void setJointSeq(int graspingState, cnoid::VectorXd& jointSeq, ArmPtr& arm_, FingerPtr fingers_[]);
		void setGraspingStateSeq(int graspingState, int graspingState2, int contactState);
		void setTargetObjPos(cnoid::Vector3& P_ini, cnoid::Matrix3& R_ini, std::vector<cnoid::Vector3>& P_des, std::vector<cnoid::Matrix3>& R_des, int j);
		bool withinJointLimit();
		bool isColliding(int graspingState, int graspingState2);

		cnoid::BodyItemPtr envItem;
		ArmPtr arm_g, arm_p; 
		FingerPtr fingers_g[2], fingers_p[2];
		bool firstPick;
		cnoid::Vector3 Pinit;
		//std::vector<double> motionTimeSeq;
		cnoid::Vector3 approachVec, approachVec2;
		bool movePallet();
		bool pushBox();
		cnoid::Vector3 visOffset;

		CollisionPair *cp;
		ParameterFileData *pf;
};

}
}

#endif
