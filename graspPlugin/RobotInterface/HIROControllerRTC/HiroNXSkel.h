// -*- C++ -*-
/*!
 *
 * THIS FILE IS GENERATED AUTOMATICALLY!! DO NOT EDIT!!
 *
 * @file HiroNXSkel.h 
 * @brief HiroNX server skeleton header wrapper code
 * @date Fri Oct 28 18:38:38 2011 
 *
 */

#ifndef HIRONXSKEL_H
#define HIRONXSKEL_H



#include <rtm/config_rtc.h>
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION

#if   defined ORB_IS_TAO
#  include "HiroNXC.h"
#  include "HiroNXS.h"
#elif defined ORB_IS_OMNIORB
#  if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#    undef USE_stub_in_nt_dll
#  endif
#  include "HiroNX.hh"
#elif defined ORB_IS_MICO
#  include "HiroNX.h"
#elif defined ORB_IS_ORBIT2
#  include "/HiroNX-cpp-stubs.h"
#  include "/HiroNX-cpp-skels.h"
#else
#  error "NO ORB defined"
#endif

#endif // HIRONXSKEL_H
