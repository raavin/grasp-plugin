// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
  c) Kensuke Harada (AIST)
*/

#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MenuManager>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/SceneView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include <osg/Camera>
#include <osg/Geode>
#include <osg/GLExtensions>
#include <osgDB/ReadFile>
#include <osgGA/GUIEventAdapter>
using namespace osgGA;

#include <cnoid/SceneBodyManager>

#include "ManipBar.h"
#include "ManipController.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
using namespace grasp::GripperManipulation;

Vector3 objVisPos;
Matrix3 objVisRot;
double objMass;
string bodywrlname;

class ManipSceneBody : public cnoid::SceneBody {
public:
	ManipSceneBody(cnoid::BodyItemPtr bodyItem) : SceneBody(bodyItem), mes(*MessageView::mainInstance()), os (MessageView::mainInstance()->cout()) {
		pointedItem = bodyItem;
	}
	
protected:
	
	virtual ~ManipSceneBody() {  };
	
	virtual bool onButtonPressEvent( const SceneViewEvent& event );
	

private:
	MessageView& mes;
	std::ostream& os;

	bool isDragging;
	cnoid::BodyItemPtr pointedItem;
	
};
typedef osg::ref_ptr<ManipSceneBody> ManipSceneBodyPtr;

bool ManipSceneBody::onButtonPressEvent( const SceneViewEvent& event )
{

  if( ( event.modKeyMask() & GUIEventAdapter::MODKEY_CTRL )
		  && ( event.button() == GUIEventAdapter::LEFT_MOUSE_BUTTON ) ){	/* modified by qtconv.rb 3rd rule*/
		cnoid::Vector3 pressPos = Vector3( event.point().x(), event.point().y(), event.point().z() );

		PlacePlanner::instance()->findTargetTriangle( getPointedSceneLink(), pressPos, pointedItem );

		return true;
    }

    return cnoid::SceneBody::onButtonPressEvent( event );
}

cnoid::SceneBody* create(cnoid::BodyItem* item) {
	return new ManipSceneBody(item);;
}


class ManipPlugin : public cnoid::Plugin
{
private:

	//SceneView* sceneView;
	
	bool onTimeout() {
		
		return true;
	}
	
public:
	
	ManipPlugin() : Plugin("GripperManipulation") {
		depend("Grasp");
	}
	
	virtual bool initialize() {
		cnoid::SceneBodyManager* manager = cnoid::SceneBodyManager::instance();
		grasp::GraspController::instance(ManipController::instance());
		//ManipRtc::instance()->RtcStart();
		//manage(manager->addSceneBodyFactory(create));
		manage(manager->addSceneBodyFactory(create));

		addToolBar(ManipBar::instance());
		
		return true;
	}
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(ManipPlugin);
