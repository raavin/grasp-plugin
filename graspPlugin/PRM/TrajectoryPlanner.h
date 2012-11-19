/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _TrajectoryPlanner_H
#define _TrajectoryPlanner_H

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <../src/PoseSeqPlugin/PoseSeqItem.h>  
#include <cnoid/BodyMotion>  	/* modified by qtconv.rb 0th rule*/  

#include <cnoid/EigenTypes>	/* modified by qtconv.rb 0th rule*/  

#include "exportdef.h"

#include "../Grasp/PlanBase.h"

namespace grasp{


class TrajectoryPlanner {

	public :

	TrajectoryPlanner(int id=0);
	virtual ~TrajectoryPlanner(){};
		
	virtual void setStartPos();
	virtual bool doTrajectoryPlanning();
		
	virtual bool updateTrajectoryFromMotion(const cnoid::BodyMotionPtr motionObject, const cnoid::BodyMotionPtr motionRobot, std::vector<MotionState>& motionSeq);
	virtual bool outputTrajectoryForOpenHRP(const cnoid::BodyMotionPtr motionRobot);
	
	cnoid::PoseSeqItem* poseSeqItemRobot;
	cnoid::PoseSeqItem* poseSeqItemObject;

//	hrp::Link *palm;
//	hrp::Link *base;
	cnoid::VectorXd iniJoint, finJoint;
	std::vector<MotionState> motionSeq;
		
};


}


#endif
