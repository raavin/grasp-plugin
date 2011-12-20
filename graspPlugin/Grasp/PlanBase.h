/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _PlanBase_H
#define _PlanBase_H

#include <stdlib.h>
#include <time.h>
#include <cnoid/Item>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include "GraspPluginManager.h"

#include "Finger.h"
#include "Arm.h"

#include "exportdef.h"

namespace grasp{

class EXCADE_API PlanBase;
	
class Box{
	public:
		cnoid::Vector3 p;
		cnoid::Matrix3 R;
		cnoid::Vector3 edge;
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
	MotionState(){ }
	MotionState(cnoid::VectorXd jointSeq, int graspingState=0, int graspingState2=0, int id=-1, double tolerance=-1, double time=0){
		this->jointSeq = cnoid::VectorXd::Zero(jointSeq.size());
		this->jointSeq = jointSeq;
		this->graspingState = graspingState;
		this->graspingState2= graspingState2;
		this->time = time;
		this->id = id;
		this->tolerance = tolerance;
	}

	cnoid::VectorXd jointSeq;
	int graspingState, graspingState2;
	int objectContactState;
	double time;
	std::vector<int> pathPlanDOF;
	std::vector<double> motionTimeSeq;
	int id;
	double tolerance;
	//cnoid::VectorXd bodyJointSeq;
	//std::vector<int>bodyJointPtrs; //this is id for armJointSeq;

	private:
};


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
	std::list<TargetObject*> multiTargetObject;
	
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

	void setGraspingState(int state);
	void setGraspingState2(int state);
	void setObjectContactState(int state) { objectContactState = state; }
	
	int getGraspingState() {return graspingState; }
	int checkAllGraspingState() {return (graspingState > graspingState2) ? graspingState : graspingState2; }
	int getGraspingState2() {return graspingState2; }
	int getObjectContactState() {return objectContactState; }
	void setTrajectoryPlanDOF();
	void setTrajectoryPlanDOF(int k);
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

	
	std::vector<cnoid::ColdetLinkPairPtr> robotSelfPairs, robotEnvPairs, robotObjPairs, objEnvPairs; // armEnvPairs;
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
