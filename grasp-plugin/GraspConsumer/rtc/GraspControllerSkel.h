// -*- C++ -*-
/*!
 *
 * THIS FILE IS GENERATED AUTOMATICALLY!! DO NOT EDIT!!
 *
 * @file GraspControllerSkel.h 
 * @brief GraspController server skeleton header wrapper code
 * @date Thu Nov  4 14:14:14 2010 
 *
 */

#ifndef GRASPCONTROLLERSKEL_H
#define GRASPCONTROLLERSKEL_H



#include <rtm/config_rtc.h>
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION

#if   defined ORB_IS_TAO
#  include "GraspControllerC.h"
#  include "GraspControllerS.h"
#elif defined ORB_IS_OMNIORB
#  if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#    undef USE_stub_in_nt_dll
#  endif
#  include "GraspController.hh"
#elif defined ORB_IS_MICO
#  include "GraspController.h"
#elif defined ORB_IS_ORBIT2
#  include "/GraspController-cpp-stubs.h"
#  include "/GraspController-cpp-skels.h"
#else
#  error "NO ORB defined"
#endif

#endif // GRASPCONTROLLERSKEL_H
