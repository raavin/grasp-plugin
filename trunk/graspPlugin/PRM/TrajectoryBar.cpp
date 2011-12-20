/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "TrajectoryBar.h"
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/  
#include <boost/bind.hpp>
#include <boost/format.hpp>
// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/  

#include "../Grasp/GraspController.h"
//#include "TrajectoryPlanner.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

TrajectoryBar* TrajectoryBar::instance()
{
	static TrajectoryBar* instance = new TrajectoryBar();
	return instance;
}

TrajectoryBar::TrajectoryBar()
	: ToolBar("TrajectoryBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{
	
	addSeparator();
	
	addLabel(("=PathPlan="));

	addButton(("Start"), ("Path planning start"))->
		sigClicked().connect(bind(&TrajectoryBar::onTrajectoryPlanButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("setStartState"), ("Set start Motion"))->
		sigClicked().connect(bind(&TrajectoryBar::onSetStartMotionStateButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  
	
	addButton(("setEndState"), ("Set End Motion"))->
		sigClicked().connect(bind(&TrajectoryBar::onSetEndMotionStateButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addSeparator();

	// show_all_children();	/* modified by qtconv.rb 7th rule*/  

}


TrajectoryBar::~TrajectoryBar()
{
//	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
}




void TrajectoryBar::onTrajectoryPlanButtonClicked()
{
	if(PlanBase::instance()->bodyItemRobot()==NULL){
		os << "error: Set Robot" << endl;
		return;
	}
	trajectoryPlanner_ = new TrajectoryPlanner();

	bool ret = trajectoryPlanner_->doTrajectoryPlanning();

	if(ret)
			os <<  "Trajectory Planning is finished" << endl;	
	else
			os <<  "Trajectory Planning is failed" << endl;	
}

void TrajectoryBar::onSetStartMotionStateButtonClicked(){
	if(PlanBase::instance()->bodyItemRobot()==NULL){
		os << "error: Set startMotionState" << endl;
		return;
	}
	PlanBase::instance()->startMotionState = PlanBase::instance()->getMotionState();
}
void TrajectoryBar::onSetEndMotionStateButtonClicked(){
	if(PlanBase::instance()->bodyItemRobot()==NULL){
		os << "error: Set endMotionState" << endl;
		return;
	}
	PlanBase::instance()->endMotionState = PlanBase::instance()->getMotionState();
}


bool TrajectoryBar::storeState(Archive& archive)
{
//	if(currentBodyItem_){
//	/	archive.writeItemId("current", currentBodyItem_);
//	}
	return true;
}


bool TrajectoryBar::restoreState(const Archive& archive)
{
//	if(!currentBodyItem_){
	//	currentBodyItem_ = archive.findItem<BodyItem>("current");
		//if(currentBodyItem_){
		//	if(targetBodyItems.empty()){
		//		targetBodyItems.push_back(currentBodyItem_);
			//}
			//sigCurrentBodyItemChanged_(currentBodyItem_.get());
		//}
//	}
	return true;
}
