/**
  @file
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <boost/bind.hpp>

int talker();

using namespace boost;
using namespace cnoid;

class HelloWorldPlugin : public Plugin
{
    void onHelloWorldActivated() {
        MessageView::mainInstance()->putln(tr("Hello World !"));
	talker();
    }

public:
    
    HelloWorldPlugin() : Plugin("HelloWorld") { }
    
    virtual bool initialize() {

        menuManager().setPath(tr("/View")).addItem(tr("Hello World"))
            ->sigTriggered().connect(
                bind(&HelloWorldPlugin::onHelloWorldActivated, this));
        
        return true;
    }
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(HelloWorldPlugin)
