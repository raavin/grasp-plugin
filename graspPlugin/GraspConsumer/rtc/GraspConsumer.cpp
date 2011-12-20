// -*- C++ -*-
/*!
 * @file  GraspConsumer.cpp
 * @brief Grasp Consumer
 * @date $Date$
 *
 * $Id$
 */

#include "GraspConsumer.h"

// Module specification
// <rtc-template block="module_spec">
static const char* graspconsumer_spec[] =
  {
    "implementation_id", "GraspConsumer",
    "type_name",         "GraspConsumer",
    "description",       "Grasp Consumer",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "TestInterfade",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
GraspConsumer::GraspConsumer(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_PlannStartPortPort("PlannStartPort"),
    m_ResultPortPort("ResultPort")

    // </rtc-template>
{

}

/*!
 * @brief destructor
 */
GraspConsumer::~GraspConsumer()
{
}



RTC::ReturnCode_t GraspConsumer::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_ResultPortPort.registerProvider("Result", "GraspPlanResult", m_Result);
  
  // Set service consumers to Ports
  m_PlannStartPortPort.registerConsumer("PlanStart", "GraspPlanStart", m_PlanStart);
  
  // Set CORBA Service Ports
  addPort(m_PlannStartPortPort);
  addPort(m_ResultPortPort);
  
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t GraspConsumer::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t GraspConsumer::onExecute(RTC::UniqueId ec_id)
{
//  const char* userid = "***";
//  const char* password = "***";
/*
	std::string area, point;

  std::cout << std::endl;
//  std::cout << " [0: Arm Motion, 1: Arm Flag Reset]" << std::endl;
  std::cout << " [0: execute]" << std::endl;
	
	int i;
	std::cin >> i;
	std::cout << std::endl; 
	
	if(i)   return RTC::RTC_OK;
	
	CORBA::ULong state; 

	GraspPlanStart::DblSequence3 objPos;
	GraspPlanStart::DblSequence9 objOri;
	CORBA::ULong ObjId=2;

	objPos.length(3);
	objOri.length(9);


	for(unsigned int i=0; i<3; i++){
	    objPos[i] = 0;
	    for(unsigned int j=0; j<3; j++){
		    objOri[3*i+j] = 0;
		    if(i==j) objOri[3*i+j] = 1;
	    }
	}
	m_PlanStart->GraspPlanningStart(ObjId, objPos, objOri, state);
*/	
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t GraspConsumer::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void GraspConsumerInit(RTC::Manager* manager)
  {
    coil::Properties profile(graspconsumer_spec);
    manager->registerFactory(profile,
                             RTC::Create<GraspConsumer>,
                             RTC::Delete<GraspConsumer>);
  }
  
};


