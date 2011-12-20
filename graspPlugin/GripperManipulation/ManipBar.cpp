// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Kensuke Harada (AIST)
*/

#include "ManipBar.h"
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include "ManipController.h"

#ifdef EXPO_DEMO
#include "../PRM/TrajectoryPlanner.h"
#endif

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp::GripperManipulation;

ManipBar* ManipBar::instance()
{
	static ManipBar* instance = new ManipBar();
	return instance;
}

ManipBar::ManipBar()
	: ToolBar("ManipBar"),
	  mes(*MessageView::mainInstance()),
	  os (MessageView::mainInstance()->cout() )
{
	
	addSeparator();
#ifdef EXPO_DEMO
	addLabel(("=Manip="));
	
	addButton(("Start"), ("Pick and Place Planning"))->
		sigClicked().connect(bind(&ManipBar::onStartButtonClicked, this));	/* modified by qtconv.rb 6th rule*/
#endif
	addLabel(("=INTENTION="));
	addButton(("1"), ("First Part of Object"))->
		sigClicked().connect(bind(&ManipBar::onFirstButtonClicked, this));	/* modified by qtconv.rb 6th rule*/
	
	addButton(("2"), ("Second Part of Object"))->
		sigClicked().connect(bind(&ManipBar::onSecondButtonClicked, this));	/* modified by qtconv.rb 6th rule*/
	
	addButton(("3"), ("Third Part of Object"))->
		sigClicked().connect(bind(&ManipBar::onThirdButtonClicked, this));	/* modified by qtconv.rb 6th rule*/
#ifdef EXPO_DEMO
	addButton(("4"), ("Fourth Part of Object"))->
		sigClicked().connect(bind(&ManipBar::onFourButtonClicked, this));	/* modified by qtconv.rb 6th rule*/	
#endif
	addSeparator();
	
	// show_all_children();	/* modified by qtconv.rb 7th rule*/
}


ManipBar::~ManipBar()
{
}

#ifdef EXPO_DEMO
void ManipBar::onStartButtonClicked()
{
	ManipController::instance()->initial(PlanBase::instance()->targetObject,  PlanBase::instance()->targetArmFinger);
	ManipController::instance()->doGraspPlanning();
	TrajectoryPlanner tp;
	tp.doTrajectoryPlanning();
	
}
#endif

void ManipBar::onAllButtonClicked()
{
}

void ManipBar::onFirstButtonClicked()
{
	ManipController::instance()->intention = 0;
	os <<  "Intention is set to 0" << endl;
}

void ManipBar::onSecondButtonClicked()
{
	ManipController::instance()->intention = 1;
	os <<  "Intention is set to 1" << endl;
}

void ManipBar::onThirdButtonClicked()
{
	ManipController::instance()->intention = 2;
	os <<  "Intention is set to 2" << endl;
}

void ManipBar::onFourButtonClicked()
{
	ManipController::instance()->intention = 3;
	os <<  "Intention is set to 3" << endl;
}


bool ManipBar::storeState(Archive& archive)
{
	return true;
}

bool ManipBar::restoreState(const Archive& archive)
{
	return true;
}
