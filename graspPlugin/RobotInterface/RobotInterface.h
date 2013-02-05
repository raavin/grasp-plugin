#ifndef _ROBOTINTERFACE_H
#define _RROBOTINTERFACE_H

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>
#include <cnoid/MessageView>
#include <cnoid/EigenUtil>
#include "ArmControllerRtc.h"
#include "../Grasp/GraspController.h"
#include "../PickAndPlacePlanner/RobotLocalFunctions.h"

namespace grasp{

class RobotInterface {

	public :

	RobotInterface();
	virtual ~RobotInterface(){};
/*
	static RobotInterface* instance(){
		static RobotInterface* instance = new RobotInterface();
		return instance;
	}
*/
	static RobotInterface* instance( RobotInterface* ri=NULL )
	{
		static RobotInterface* instance = (ri) ? ri : new RobotInterface();
		if(ri) instance = ri;
		return instance;
	};

	virtual bool runPythonCommand(std::string func);

	virtual void doReadFromFile();
	virtual void doRecoginitionAndRead();
	virtual void doCapture();
	virtual void doJntCalib();
	virtual void doSrvOn();
	virtual void doSrvOff();
	virtual void doHome();
	virtual void doMove();

	bool isMulti;

	protected :

	//virtual int objName2Id(std::string objname, std::string obj_data);
	//bool moveUp;

	private :
		bool isSingleArm() { return PlanBase::instance()->armsList.size() == 1; }
		bool isDualArm() { return PlanBase::instance()->armsList.size() == 2; }
		int moveDualArm();
		int moveSingleArm();
		::ArmController * controllerRtc() { return ArmControllerRtc::instance()->comp_; };
		int numJoints() { return PlanBase::instance()->arm()->arm_path->numJoints(); }
		//void writeJointSeq(char * jointlogfile, const unsigned int filelength, const std::vector<cnoid::VectorXd>& jointSeq, std::vector<double>& motionTimeSeq);
		void convertAngles(const cnoid::VectorXd & seq, std::vector<double> & angles);
		void checkAngles(std::vector<double> & angles);
};



}


#endif
