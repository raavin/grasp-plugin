/*!
  @file
  @author Tokuo Tsuji
*/

#include "ImageWidgetView.h"
#include <cnoid/Plugin>
#include <cnoid/MessageView>

#include <iostream>

using namespace std;

using namespace cnoid;

namespace {
    
    class ImageWidgetPlugin : public Plugin
    {
    public:
	ImageWidgetPlugin() : Plugin("ImageWidget")
	{ 

        }
        
        virtual ~ImageWidgetPlugin()
        {

        }

        virtual bool initialize() {

		addView(new grasp::ImageWidgetView());
		return true;
        }
        
        virtual bool finalize() {
		return true;
        }
        
    private:
        //AudioItemManager* audioItemManager;
    };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ImageWidgetPlugin);
