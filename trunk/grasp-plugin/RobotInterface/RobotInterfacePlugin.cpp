/*! @file
  @author Tokuo Tsuji
*/

#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/
//#include <cnoid/MenuManager>	/* modified by qtconv.rb 0th rule*/
//#include <cnoid/SceneView>	/* modified by qtconv.rb 0th rule*/
//#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/

#include "RobotInterfaceBar.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
#include "HIROControllerRTC/HIROController.h"
#include "HIROControllerRtc.h"


class RobotInterfacePlugin : public cnoid::Plugin
{
private:

//	SceneView* sceneView;

	bool onTimeout() {

		return true;
	}

public:

	RobotInterfacePlugin() : Plugin("RobotInterface") {
		depend("Grasp");
	}

	virtual bool initialize() {
		addToolBar(RobotInterfaceBar::instance());
		HIROControllerRtc::instance()->RtcStart();

		return true;
	}
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(RobotInterfacePlugin);
