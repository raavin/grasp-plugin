// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <iostream>
#include "GraspBar.h"
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/  
#include <boost/bind.hpp>
#include <boost/format.hpp>
// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/  

#include "GraspController.h"
#include "PlanInterface.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

GraspBar* GraspBar::instance()
{
	static GraspBar* instance = new GraspBar();
	return instance;
}

GraspBar::GraspBar()
	: ToolBar("GraspBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{
	
	addSeparator();
	
	addLabel(("=Planner="));
	
	addButton(("SetObject"), ("Set Selected bodyitem as a grasped object"))->
		sigClicked().connect(bind(&GraspBar::onObjectButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("SetRobot"), ("Set the preset bodyitem as a grasping robot"))->
		sigClicked().connect(bind(&GraspBar::onRobotButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("Grasp"), ("Grasp planning start"))->
		sigClicked().connect(bind(&GraspBar::onGraspButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("Place"), ("Place planning start"))->
		sigClicked().connect(bind(&GraspBar::onPlaceButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("Pick&Place"), ("Pick and place planning start"))->
		sigClicked().connect(bind(&GraspBar::onPickAndPlaceButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  
	
	addButton(("Stop"), ("Grasp planning stop"))->
		sigClicked().connect(bind(&GraspBar::onStopButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addSeparator();

	addButton(("SetEnv"), ("Set the preset bodyitem as a sorrounding environment"))->
		sigClicked().connect(bind(&GraspBar::onEnvironButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("RemoveEnv"), ("Remove the preset bodyitem from a sorrounding environment"))->
		sigClicked().connect(bind(&GraspBar::onRemoveEnvButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  
	
	addSeparator();

	addButton(("SaveGraspPattern"), ("Save grasp pattern"))->
		sigClicked().connect(bind(&GraspBar::onSaveGPButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  
		
	addButton(("SelectGraspPattern"), ("Load selected grasp pattern"))->
		sigClicked().connect(bind(&GraspBar::onSelectGraspPattern, this));	/* modified by qtconv.rb 6th rule*/  
		
	addButton(("DisplayGRC"), ("Display GRC Position"))->
		sigClicked().connect(bind(&GraspBar::onDisplayGRCPositionButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("CloseFingers"), ("Close Fingers"))->
		sigClicked().connect(bind(&GraspBar::onCloseButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  
	
	addSeparator();
	// show_all_children();	/* modified by qtconv.rb 7th rule*/  

	ItemTreeView::mainInstance()->sigSelectionChanged().connect(
		bind(&GraspBar::onItemSelectionChanged, this, _1));
}


GraspBar::~GraspBar()
{
	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
}


/**
   \todo ItemTreeView::sigSelectionChanged() should be emitted
   after the final selection state has been determined.
*/
bool GraspBar::makeSingleSelection(BodyItemPtr bodyItem)
{
	ItemTreeView* tree = ItemTreeView::mainInstance()->mainInstance();

	ItemList<BodyItem> prevSelected = selectedBodyItems_;

	for(size_t i=0; i < prevSelected.size(); ++i){
		BodyItem* item = prevSelected[i];
		if(item != bodyItem && tree->isItemSelected(item)){
			tree->selectItem(item, false);
		}
	}

	bool isSelected = tree->isItemSelected(bodyItem);
	if(!isSelected){
		isSelected = tree->selectItem(bodyItem, true);
	}

	return isSelected;
}


void GraspBar::onItemSelectionChanged(const ItemList<BodyItem>& bodyItems)
{
	bool selectedBodyItemsChanged = false;
	
	if(selectedBodyItems_ != bodyItems){
		selectedBodyItems_ = bodyItems;
		selectedBodyItemsChanged = true;
	}

	BodyItemPtr firstItem = bodyItems.toSingle();

	if(firstItem && firstItem != currentBodyItem_){
		currentBodyItem_ = firstItem;
		connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
		connectionOfCurrentBodyItemDetachedFromRoot = currentBodyItem_->sigDetachedFromRoot().connect(
			bind(&GraspBar::onBodyItemDetachedFromRoot, this));
		sigCurrentBodyItemChanged_(currentBodyItem_.get());
	}

	if(selectedBodyItemsChanged){
		sigBodyItemSelectionChanged_(selectedBodyItems_);
	}

	targetBodyItems.clear();
	if(selectedBodyItems_.empty()){
//		if(currentBodyItem_){
//			targetBodyItems.push_back(currentBodyItem_);
//		}
	} else {
		targetBodyItems = selectedBodyItems_;
	}
}


void GraspBar::onBodyItemDetachedFromRoot()
{
	currentBodyItem_ = 0;
	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
	sigCurrentBodyItemChanged_(0);
}


void GraspBar::onObjectButtonClicked()
{
	if(targetBodyItems.size()==1){
		PlanBase::instance()->SetGraspedObject(targetBodyItems[0]);
		os << PlanBase::instance()->targetObject->bodyItemObject->name() << " is grasped object"<< endl;
	}else{
		os <<  "Please selecet one bodyitem" << endl;	
	}
	
}

void GraspBar::onRobotButtonClicked()
{
	if(PlanBase::instance()->targetArmFinger){
		if( ( targetBodyItems.size()==1 && PlanBase::instance()->bodyItemRobot()==targetBodyItems[0] )  ||  targetBodyItems.size()==0){
			PlanBase::instance()->targetArmFinger = PlanBase::instance()->armsList[(PlanBase::instance()->targetArmFinger->id+1)%PlanBase::instance()->armsList.size()];
			os  << PlanBase::instance()->targetArmFinger->name << " is target arm"<< endl;
			return;
		}
	}
	if(targetBodyItems.size()==1){
		if( PlanBase::instance()->SetGraspingRobot(targetBodyItems[0]) ){		
			os << PlanBase::instance()->bodyItemRobot()->name() << " is grasping robot"<< endl;
			os  << PlanBase::instance()->targetArmFinger->name << " is target arm"<< endl;
		}
	}else{
		os <<  "Please selecet one bodyitem" << endl;	
	}
}

void GraspBar::onEnvironButtonClicked()
{
	if(targetBodyItems.size()>0){
		for(unsigned int i=0;i<targetBodyItems.size();i++){
			PlanBase::instance()->SetEnvironment(targetBodyItems[i]);
			//if(PlanBase::instance()->objTag2Item.find(targetBodyItems[i]->name()) == PlanBase::instance()->objTag2Item.end()){ 
			//	PlanBase::instance()->objTag2Item.insert( pair <string,BodyItemPtr>(targetBodyItems[i]->name(), targetBodyItems[i]) );
			//}
			os << targetBodyItems[i]->name() << " is sorrounding environment"<< endl;
		}
	}else{
		os <<  "Please selecet more than one bodyitem" << endl;	
	}
}

void GraspBar::onRemoveEnvButtonClicked()
{
	if(targetBodyItems.size()>0){
		for(unsigned int i=0;i<targetBodyItems.size();i++){
			PlanBase::instance()->RemoveEnvironment(targetBodyItems[i]);
			os << targetBodyItems[i]->name() << " is removed from environment"<< endl;
		}
	}else{
		os <<  "Please selecet more than one bodyitem" << endl;	
	}
}


void GraspBar::onGraspButtonClicked()
{
	bool init = PlanBase::instance()->initial();
	if(!init){
		os << "Failed: Grasp Planning Initial" << endl;
		return;
	}
	
	try{
		PlanInterface::instance()->doGraspPlanning();
	}
	catch(int number){
		PlanBase::instance()->stopFlag=false;
		os <<  "Grasp Planning is stopped" << endl;
	}
	PlanBase::instance()->flush();
//	cout << "test" << endl;
}

void GraspBar::onPlaceButtonClicked()
{
	try{
		PlanInterface::instance()->doPlacePlanning();
	}
	catch(int number){
		PlanBase::instance()->stopFlag=false;
		os <<  "Place Planning is stopped" << endl;
	}
	PlanBase::instance()->flush();
}

void GraspBar::onPickAndPlaceButtonClicked()
{
	try{
		PlanInterface::instance()->doPickAndPlacePlanning();
	}
	catch(int number){
		PlanBase::instance()->stopFlag=false;
		os <<  "Grasp Planning is stopped" << endl;
	}
	PlanBase::instance()->flush();
}

void GraspBar::onStopButtonClicked()
{
	PlanBase::instance()->stopFlag = true;
	os <<  "Stop button is pressed" << endl;	
}


void GraspBar::onSaveGPButtonClicked(){
	GraspController::instance()->saveGraspPattern();
}

void GraspBar::onSelectGraspPattern(){
	GraspController::instance()->loadAndSelectGraspPattern();
}

void GraspBar::onDisplayGRCPositionButtonClicked(){
	GraspController::instance()->doDisplayGRCPosition();
}



void GraspBar::onCloseButtonClicked()
{
	GraspController::instance()->closeFingers();
}

bool GraspBar::storeState(Archive& archive)
{
	PlanBase* gc = PlanBase::instance();
	if(gc->targetArmFinger){
       archive.writeItemId("graspRobot", gc->bodyItemRobot());
	}
	if(gc->targetObject){
       archive.writeItemId("graspObject", gc->targetObject->bodyItemObject);
	}
	if( gc->bodyItemEnv.size() ) {
		YamlSequence& qs = *archive.createFlowStyleSequence("graspEnv");
		list<BodyItemPtr>::iterator it = gc->bodyItemEnv.begin();
		for(int i=0;i< gc->bodyItemEnv.size();i++){
			qs.append(archive.getItemId(*it), 10, i);
			it ++ ;
		}
	}
	if( gc->objTag2Item.size() ) {
		YamlSequence& qs = *archive.createFlowStyleSequence("objTag2Item");
		map<string,BodyItemPtr>::iterator it = gc->objTag2Item.begin();
		for(int i=0;i< gc->objTag2Item.size();i++){
			qs.append(archive.getItemId(it->second), 10, i);
			it++;
		}
	}		
	return true;
}


bool GraspBar::restoreState(const Archive& archive)
{
	PlanBase* gc = PlanBase::instance();
	
	BodyItemPtr bodyItem = archive.findItem<BodyItem>("graspRobot");
	if(bodyItem) gc->SetGraspingRobot(bodyItem);
	
	bodyItem = archive.findItem<BodyItem>("graspObject");
	if(bodyItem) gc->SetGraspedObject(bodyItem);
	
	const YamlSequence& qs = *archive.findSequence("graspEnv");
	if(qs.isValid()){
		gc->bodyItemEnv.clear();
		for(int i=0; i < qs.size(); ++i){
			BodyItemPtr bodyItem = archive.findItem<BodyItem>(qs[i].toInt());
			gc->bodyItemEnv.push_back( bodyItem );
		}
	}
   
	const YamlSequence& qs2 = *archive.findSequence("objTag2Item");
	if(qs2.isValid()){
		gc->objTag2Item.clear();
		for(int i=0; i < qs2.size(); ++i){
			BodyItemPtr bodyItem = archive.findItem<BodyItem>(qs2[i].toInt());
			string tagId = bodyItem->name();
			gc->objTag2Item.insert( pair <string,BodyItemPtr>(tagId, bodyItem) );
		}
	}
	
	const YamlSequence& qs3 = *archive.findSequence("GraspPlanning");
	if(qs3.isValid()){
		bool init = PlanBase::instance()->initial();
		if(!init){
			os << "Failed: Grasp Planning Initial" << endl;
			return true;
		}
		
		try{
			PlanInterface::instance()->doGraspPlanning();
		}
		catch(int number){
			PlanBase::instance()->stopFlag=false;
			os <<  "Grasp Planning is stopped" << endl;
		}
		PlanBase::instance()->flush();
	}
	
	return true;
}
