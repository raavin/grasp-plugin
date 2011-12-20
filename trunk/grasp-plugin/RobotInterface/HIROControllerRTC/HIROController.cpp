// -*- C++ -*-
/*!
 * @file  HIROController.cpp
 * @brief Control HIRO Robot from GripperManipulation
 * @date $Date$
 *
 * $Id$
 */

#include "HIROController.h"

// Module specification
// <rtc-template block="module_spec">
static const char* hirocontroller_spec[] =
  {
    "implementation_id", "HIROController",
    "type_name",         "HIROController",
    "description",       "Control HIRO Robot from GripperManipulation",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "VMRG",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
HIROController::HIROController(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_HIROPort("HIRO"),
    m_HiroNXPort("HiroNX")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
HIROController::~HIROController()
{
}



RTC::ReturnCode_t HIROController::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer

  // Set service provider to Ports

  // Set service consumers to Ports
  m_HIROPort.registerConsumer("common", "CommonCommands", m_common);
  m_HIROPort.registerConsumer("motion", "MotionCommands", m_motion);
  m_HiroNXPort.registerConsumer("manipulator", "HiroNX", m_manipulator);

  // Set CORBA Service Ports
  addPort(m_HiroNXPort);
  addPort(m_HIROPort);

  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t HIROController::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HIROController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HIROController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HIROController::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HIROController::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HIROController::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HIROController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HIROController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HIROController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HIROController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HIROController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void HIROControllerInit(RTC::Manager* manager)
  {
    coil::Properties profile(hirocontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<HIROController>,
                             RTC::Delete<HIROController>);
  }

};


