
/*! @file
  @author Shin'ichiro Nakaoka
*/

// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/  
#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ItemManager>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/BodyMotionItem>	/* modified by qtconv.rb 0th rule*/  

#include <rtm/Manager.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include "GraspConsumer.h"

#include "GraspRtcBar.h"
#include "GraspRtcController.h"


using namespace cnoid;
using namespace cnoid;
using namespace grasp;




namespace {
    class GraspRtcPlugin : public Plugin
    {
    public:
        
        GraspRtcPlugin() : Plugin("Rtc") {
            depend("Grasp");
        }

        bool initialize() {
	
			GraspRtcController::instance()->RtcStart ();
			addToolBar(grasp::GraspRtcBar::instance());
            
            return true;
        }
    };
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(GraspRtcPlugin);
