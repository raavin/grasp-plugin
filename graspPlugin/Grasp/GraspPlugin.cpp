/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <iostream>

#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/  

#include "PlanBase.h"
#include "GraspSceneBody.h"
#include "GraspBar.h"
#include "PlaceController.h"


using namespace std;
using namespace cnoid;
using namespace grasp;



/*
bool GraspSceneBody::onKeyPressEvent(const SceneViewEvent& event)
{
	bool handled = true;

	switch(std::toupper(event.key())){
	case 'G':
		cout << "test " << endl;

	default:
		handled = false;
	return cnoid::SceneBody::onKeyPressEvent(event);
		break;
	}
		
	return handled;
}
*/

bool GraspSceneBody::onButtonPressEvent(const SceneViewEvent& event) {
    bool handled = false;

	if(event.button() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON){	/* modified by qtconv.rb 3rd rule*/  
		
		Vector3 pressPos = cnoid::Vector3(event.point().x(),event.point().y(),event.point().z());
		Vector3 normal = cnoid::Vector3(event.normal().x(),event.normal().y(),event.normal().z());
		
		PlaceController::instance()->setTargetPose(pressPos,normal);
			
		cout << pressPos << endl;
		cout << normal << endl;
	}
	return cnoid::SceneBody::onButtonPressEvent(event);
}


cnoid::SceneBody* GraspSceneBody::create(cnoid::BodyItem* item)
{
	return new GraspSceneBody(item);
}

class GrasplotPlugin : public cnoid::Plugin
{
private:

	SceneView* sceneView;
	
	bool onTimeout() {
	   
		return true;
	}

public:
	
	GrasplotPlugin() : Plugin("Grasp") { 
		depend("Body");
	}
	
	virtual bool initialize() {

	
		cnoid::SceneBodyManager* manager = cnoid::SceneBodyManager::instance();
		manage(manager->addSceneBodyFactory(GraspSceneBody::create));
	
		addToolBar(GraspBar::instance());

			
		return true;
	}
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(GrasplotPlugin);
