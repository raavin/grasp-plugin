#ifndef _ROBOTINTERFACE_H
#define _RROBOTINTERFACE_H

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/
#include "ArmControllerRtc.h"


using namespace std;

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

	virtual bool runPythonCommand(string func);

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

	virtual int objName2Id(string objname, string obj_data);
	bool moveUp;

	private :
		bool isSingleArm() { return PlanBase::instance()->armsList.size() == 1; }
		bool isDualArm() { return PlanBase::instance()->armsList.size() == 2; }
		int moveDualArm();
		int moveSingleArm();
		::ArmController * controllerRtc() { return ArmControllerRtc::instance()->comp_; };
		int numJoints() { return PlanBase::instance()->arm()->arm_path->numJoints(); }
};



}


#endif
