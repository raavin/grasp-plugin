/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _PlanBase_H
#define _PlanBase_H

#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <cnoid/Item>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/
#include "GraspPluginManager.h"

#include "Finger.h"
#include "Arm.h"

#include "exportdef.h"

#ifndef CNOID_10_11
#define YAML_SCALAR ValueNode::SCALAR
#define YAML_SEQUENCE ValueNode::SEQUENCE
#endif

namespace grasp{

class EXCADE_API PlanBase;

class Box{
	public:
		cnoid::Vector3 p;
		cnoid::Matrix3 R;
		cnoid::Vector3 edge;
};

class Approach{
	public:
		Approach(const cnoid::Vector3& dir, const std::vector<cnoid::Vector3>& pos, const std::vector<cnoid::Vector3>& nor)
		{this->direction = dir; this->position = pos; this->normal = nor;}
		cnoid::Vector3 direction;
		std::vector<cnoid::Vector3> position;
		std::vector<cnoid::Vector3> normal;
};

class EXCADE_API TargetObject{
	public:
		friend class PlanBase;

		TargetObject(cnoid::BodyItemPtr bodyItem);
		const std::string& name() { return bodyItemObject->name(); }

		cnoid::Vector3 objCoM(){ return cnoid::Vector3( objVisRot * objCoM_   + objVisPos ); }
		cnoid::Matrix3 OCP_R(){	return cnoid::Matrix3(objVisRot * OCP.R); }
		cnoid::Vector3 OCP_p(){ return cnoid::Vector3(objVisRot* OCP.p+objVisPos);	}
		cnoid::Vector3 OCP_edge(){	return cnoid::Vector3 (OCP.edge); }

		cnoid::BodyItemPtr bodyItemObject;
//	private:
		cnoid::Link *object;
		cnoid::Vector3 objVisPos;
		cnoid::Matrix3 objVisRot;
		bool offsetApplied;
		double objMass;
		cnoid::Vector3 objCoM_;
		Box OCP;
		std::string preplanningFileName;

		std::vector<Approach*> approach;
};

    class ArmFingers;
    typedef boost::intrusive_ptr<ArmFingers> ArmFingersPtr;

class InterLink{
public:
	cnoid::Link *slave;
	cnoid::Link *master;
	int slaveId;
	int masterId;
	double ratio;
};


class EXCADE_API ArmFingers  : public cnoid::Item
{
	public:

	cnoid::BodyItemPtr bodyItemRobot;
	std::string bodyItemRobotPath;
	std::string dataFilePath;

	ArmFingers(cnoid::BodyItemPtr bodyItem, const cnoid::YamlMapping& gSettings);
	int nFing;
	int nHandLink;

	cnoid::Link *palm;
	cnoid::Link *base;

	FingerPtr *fingers;
	cnoid::LinkTraverse  *handJoint;

	ArmPtr arm;
	std::string pythonInterface;

	std::string name;
	std::multimap<std::string,std::string> contactLinks;

	cnoid::Vector3 objectPalmPos;
	cnoid::Matrix3 objectPalmRot;

	int id;

	protected:
	std::ostream& os;
	GraspPluginManager gPluginManager;

};

class EXCADE_API MotionState {
public:
	MotionState(){ id = -1;}
	MotionState(cnoid::VectorXd jointSeq, int graspingState=0, int graspingState2=0, int id=-1, double tolerance=-1, double time=0){
		this->jointSeq = cnoid::VectorXd::Zero(jointSeq.size());
		this->jointSeq = jointSeq;
		this->graspingState = graspingState;
		this->graspingState2= graspingState2;
		this->time = time;
		this->id = id;
		this->tolerance = tolerance;
		pos.setZero();
		rpy.setZero();
	}

	cnoid::VectorXd jointSeq;
	cnoid::Vector3 pos;
	cnoid::Vector3 rpy;	
	int graspingState, graspingState2;
	int objectContactState;
	double time;
	std::vector<int> pathPlanDOF;
	double motionTime;
	int id;
	double tolerance;
	//cnoid::VectorXd bodyJointSeq;
	//std::vector<int>bodyJointPtrs; //this is id for armJointSeq;

	private:
};


class EXCADE_API ColdetLinkPairUpdateCheck : public cnoid::ColdetLinkPair
{
      public:
        ColdetLinkPairUpdateCheck(cnoid::Link* link1, cnoid::Link* link2) : ColdetLinkPair(link1, link2){
		checkCollisionUpdate = true;
		detectIntersectionUpdate = true;
	}

        ColdetLinkPairUpdateCheck(const ColdetLinkPairUpdateCheck& org) : ColdetLinkPair(org){
		checkCollisionUpdate = true;
		detectIntersectionUpdate = true;
	}

//        virtual ~ColdetLinkPairUpdateCheck(){};

	inline void updatePositions(){
		if( (p[0] != links[0]->p) || (p[1] != links[1]->p) || (R[0] != links[0]->R) || (R[1] != links[1]->R) ){
			p[0] = links[0]->p;
			p[1] = links[1]->p;
			R[0] = links[0]->R;
			R[1] = links[1]->R;
			checkCollisionUpdate = true;
			detectIntersectionUpdate = true;
		}
		cnoid::ColdetLinkPair::updatePositions();
	}

	inline void setTolerance(double _tolerance){
		if(tolerance()  != _tolerance){
			detectIntersectionUpdate=true;
		}
		cnoid::ColdetModelPair::setTolerance(_tolerance);
	}

        inline bool checkCollision() {
		if(checkCollisionUpdate){
			checkCollisionResult = cnoid::ColdetModelPair::checkCollision();
			checkCollisionUpdate = false;
		}
		return checkCollisionResult;
        }

        inline bool detectIntersection() {
		if(detectIntersectionUpdate){
			detectIntersectionResult = cnoid::ColdetModelPair::detectIntersection();
			detectIntersectionUpdate = false;
		}
		return detectIntersectionResult;
        }

	cnoid::Vector3 p[2];
	cnoid::Matrix3 R[2];
	bool checkCollisionResult;
	bool checkCollisionUpdate;
	bool detectIntersectionUpdate;
	bool detectIntersectionResult;
};
typedef boost::intrusive_ptr<ColdetLinkPairUpdateCheck> ColdetLinkPairUpdateCheckPtr;


class EXCADE_API PlanBase
{

public :
	PlanBase();
	~PlanBase();

    static PlanBase* instance(PlanBase *gc=NULL);

	void SetGraspedObject(cnoid::BodyItemPtr bodyItem);
	bool SetGraspingRobot(cnoid::BodyItemPtr bodyItem);

	void SetEnvironment(cnoid::BodyItemPtr bodyItem){
		bodyItemEnv.push_back(bodyItem);
		bodyItemEnv.sort();
		bodyItemEnv.unique();
	}

	void RemoveEnvironment(cnoid::BodyItemPtr bodyItem){
		bodyItemEnv.remove(bodyItem);
		bodyItemEnv.sort();
		bodyItemEnv.unique();
	}

	// interface functions of planning
	virtual bool initial();
	virtual void initialCollision();
//	virtual void runPythonScripts(){};
//	virtual void closeFingers();

	static double calcContactPoint(cnoid::ColdetLinkPairPtr cPair, cnoid::Vector3 &Po, cnoid::Vector3 &Pf, cnoid::Vector3 &objN2);
	static void calcBoundingBox(cnoid::ColdetModelPtr model, cnoid::Vector3 &edge, cnoid::Vector3& center, cnoid::Vector3& com, cnoid::Matrix3& Rot);

	//==Object==
	TargetObject* targetObject;
	std::vector<TargetObject*> multiTargetObject;

	//==Robot==
	ArmFingers* targetArmFinger;
	std::vector<ArmFingers*> armsList;
	cnoid::BodyItemPtr bodyItemRobot(){ return targetArmFinger->bodyItemRobot; }
	cnoid::BodyItemPtr bodyItemRobot(int i){ return armsList[i]->bodyItemRobot; }

	//==Env==
	std::list<cnoid::BodyItemPtr> bodyItemEnv;

	void calcForwardKinematics();
	bool isColliding();
	void setTolerance(double t){tolerance = t;}
	double clearance();
	double tolerance;
	std::vector<int> pathPlanDOF;
	std::vector<std::vector<int> > pathPlanDOFSeq;

	bool isCollidingPointCloud(const std::vector<cnoid::Vector3>& p, cnoid::BodyItemPtr item, double tol=0.001);
	bool isCollidingPointCloudFinger(const std::vector<cnoid::Vector3>& p, double tol=0.001);
	std::vector<cnoid::Vector3> pointCloud;


	void setGraspingState(int state);
	void setGraspingState2(int state);
	void setObjectContactState(int state) { objectContactState = state; }

	int getGraspingState() {return graspingState; }
	int checkAllGraspingState() {return (graspingState > graspingState2) ? graspingState : graspingState2; }
	int getGraspingState2() {return graspingState2; }
	int getObjectContactState() {return objectContactState; }
	void setTrajectoryPlanDOF();
	void setTrajectoryPlanDOF(int k);
	void setTrajectoryPlanMapDOF();	
	enum GraspingStates { NOT_GRASPING, UNDER_GRASPING, GRASPING };
	//enum FingerGraspingStates { FINGER_INIT, FINGER_CLOSE, FINGER_OPEN, FINGER_MOVE };
	enum TargetGraspingStates { ON_ENVIRONMENT, OFF_ENVIRONMENT };
	std::vector<cnoid::VectorXd> jointSeq;
	std::vector<double> motionTimeSeq;
	std::vector<int>graspingStateSeq, graspingStateSeq2, objectContactStateSeq;
	std::vector<MotionState>graspMotionSeq;
	MotionState getMotionState(double time=0);
	void setMotionState(MotionState gm);

	// MotionStateForPathPlanner
	MotionState startMotionState;
	MotionState endMotionState;
	MotionState graspMotionState;
	MotionState placeMotionState;
	cnoid::Vector3 ulimitMap;
	cnoid::Vector3 llimitMap;

	bool stopFlag;

	bool flush();
	std::ostream& os;

	cnoid::Link* object() { return targetObject->object; }
	cnoid::Vector3 objVisPos() { return  targetObject->objVisPos;  }
	cnoid::Matrix3 objVisRot() { return targetObject->objVisRot;  }
	void setObjPos(const cnoid::Vector3& P, const cnoid::Matrix3 R);

	void setVisOffset(const cnoid::Vector3& P);
	void setVisOffsetR();
	void removeVisOffset(const cnoid::Vector3& P);

	cnoid::BodyPtr body() { return targetArmFinger->bodyItemRobot->body(); }
	cnoid::BodyPtr body(int i) { return armsList[i]->bodyItemRobot->body(); }
	cnoid::Link* palm() { return targetArmFinger->palm; }
	cnoid::Link* palm(int i) { return armsList[i]->palm; }
	cnoid::Link* base() { return targetArmFinger->base; }
	cnoid::Link* base(int i) { return armsList[i]->base; }
	ArmPtr arm(){ return targetArmFinger->arm; }
	ArmPtr arm(int i){ return armsList[i]->arm; }

	FingerPtr fingers(int i) { return targetArmFinger->fingers[i]; }
	FingerPtr fingers(int i, int j) { return armsList[i]->fingers[j]; }
	int nFing(){ return targetArmFinger->nFing; };
	int nFing(int i){ return armsList[i]->nFing; };

	cnoid::LinkTraverse* handJoint() { return targetArmFinger->handJoint; }
	cnoid::LinkTraverse* handJoint(int i) { return armsList[i]->handJoint; }
	int nHandLink() {return targetArmFinger->nHandLink;}
	int nHandLink(int i) {return armsList[i]->nHandLink;}

	std::string  bodyItemRobotPath() {return  targetArmFinger->bodyItemRobotPath; }
	std::string  bodyItemRobotPath(int i) {return  armsList[i]->bodyItemRobotPath; }
	std::string  dataFilePath() {return  targetArmFinger->dataFilePath; }
	std::string  dataFilePath(int i) {return  armsList[i]->dataFilePath; }
	std::string  pythonInterface() {return  targetArmFinger->pythonInterface; }
	std::string  pythonInterface(int i) {return  armsList[i]->pythonInterface; }
	std::string armName() {return targetArmFinger->name; }
	std::string armName(int i) {return armsList[i]->name; }


	std::vector<ColdetLinkPairUpdateCheckPtr> robotSelfPairs, robotEnvPairs, robotObjPairs, objEnvPairs; // armEnvPairs;
	std::string colPairName[2], objPressName;
	cnoid::Vector3 objPressPos;

	std::map <std::string,cnoid::BodyItemPtr> objTag2Item;
	std::map <std::string,ArmFingers*> robTag2Arm;

	std::vector<InterLink> interLinkList;
	void setInterLink();

protected :

	int graspingState, graspingState2;
	int objectContactState;
};




}



#endif
