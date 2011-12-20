// -*- C++ -*-
/*!
 *
 * THIS FILE IS GENERATED AUTOMATICALLY!! DO NOT EDIT!!
 *
 * @file HiroNXStub.h 
 * @brief HiroNX client stub header wrapper code
 * @date Fri Oct 28 18:38:38 2011 
 *
 */

#ifndef HIRONXSTUB_H
#define HIRONXSTUB_H



#include <rtm/config_rtc.h>
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION

#if   defined ORB_IS_TAO
#  include "HiroNXC.h"
#elif defined ORB_IS_OMNIORB
#  if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#    undef USE_stub_in_nt_dll
#  endif
#  include "HiroNX.hh"
#elif defined ORB_IS_MICO
#  include "HiroNX.h"
#elif defined ORB_IS_ORBIT2
#  include "HiroNX-cpp-stubs.h"
#else
#  error "NO ORB defined"
#endif

#endif // HIRONXSTUB_H
