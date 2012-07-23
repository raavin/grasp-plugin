# c) Kensuke Harada (AIST)

set(libname CnoidGripperManipulationPlugin)

set(rtc-dir ./rtc)
set(grasp-dir ../grasp)
include_directories(${rtc-dir} ${grasp-dir})

set(sources
  ManipPlugin.cpp
  ManipController.cpp
  ClusterParameter.cpp
  CollisionPair.cpp
  PlacePlanner.cpp
  ManipBar.cpp
)

set(headers
    ManipBar.h
    ManipController.h
    ClusterParameter.h
    CollisionPair.h
    PlacePlanner.h
)

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
 add_library(${libname} SHARED ${sources} ${headers})
else()
 add_cnoid_plugin(${libname} SHARED ${sources} ${headers})
endif()

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)
#set_target_properties(${libname} PROPERTIES DEFINE_SYMBOL "EXCADE_SAMPLE1_MAKE_DLL")

add_definitions(-g
 -fPIC -O2 -I/usr/include -I/usr/include/rtm/idl -I.)

  target_link_libraries(${libname} CnoidBodyPlugin CnoidGraspPlugin CnoidPRMPlugin
	uuid dl pthread omniORB4 omnithread omniDynamic4 RTC coil )

apply_common_setting_for_plugin(${libname})
