/*
 * HIROControllerRtc.cpp
 *
 *  Created on: 2011-10-05
 *  Auto Generated by: gp.rb
 */

#include "HIROControllerRtc.h"

using namespace grasp;
using namespace std;
using namespace cnoid;

HIROControllerRtc* HIROControllerRtc::instance()
{
	static HIROControllerRtc* instance = new HIROControllerRtc();
	return instance;
}

int HIROControllerRtc::RtcStart()
{
	int argc= 1;
	char *argv[] = { (char *)("HIROControllerComp") };

	RTC::Manager* manager;
	manager = RTC::Manager::init(argc, argv);

	// Initialize manager
	manager->init(argc, argv);

	// Set module initialization proceduer
	// This procedure will be invoked in activateManager() function.
	manager->setModuleInitProc(MyModuleInit);

	// Activate manager and register to naming service
	manager->activateManager();

	// run the manager in blocking mode
	// runManager(false) is the default.
	//  manager->runManager();

	// If you want to run the manager in non-blocking mode, do like this
	manager->runManager(true);

	return 0;
}

void HIROControllerRtc::MyModuleInit(RTC::Manager* manager)
{
	HIROControllerInit(manager);

	// Create a component
	instance()->comp_ = (HIROController *)manager->createComponent("HIROController");
	if (instance()->comp_ ==NULL) {
		std::cerr << "Component create failed." << std::endl;
		abort();
	}

	return;
}