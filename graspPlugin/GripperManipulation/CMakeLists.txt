# c) Kensuke Harada (AIST)

set(libname CnoidGripperManipulationPlugin)

set(rtc-dir ./rtc)
set(grasp-dir ../grasp)
include_directories(${rtc-dir} ${grasp-dir})

set(sources
  ManipPlugin.cpp
  ManipController.cpp
  ClusterParameter.cpp
  RobotLocalFunctions.cpp
  CollisionPair.cpp
  PlacePlanner.cpp
  ManipBar.cpp
)

set(headers
    ManipBar.h
    ManipController.h
    ClusterParameter.h
    RobotLocalFunctions.h
    CollisionPair.h
    PlacePlanner.h
)

add_library(${libname} SHARED ${sources} ${headers} )

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)
#set_target_properties(${libname} PROPERTIES DEFINE_SYMBOL "EXCADE_SAMPLE1_MAKE_DLL")

add_definitions(-g
 -fPIC -O2 -I/usr/include -I/usr/include/rtm/idl -I.)

  target_link_libraries(${libname} CnoidBodyPlugin CnoidGraspPlugin CnoidPRMPlugin
	uuid dl pthread omniORB4 omnithread omniDynamic4 RTC coil )

  install(TARGETS ${libname}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug)

