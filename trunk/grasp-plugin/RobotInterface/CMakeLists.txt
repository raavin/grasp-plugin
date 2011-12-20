
set(hirocontroller-dir ./HIROControllerRTC)
# @author Shin'ichiro Nakaoka (AIST)

set(libname CnoidRobotInterfacePlugin)

set(sources
  RobotInterfacePlugin.cpp
  RobotInterface.cpp
  RobotInterfaceBar.cpp
  HIROControllerRtc.cpp
    ${hirocontroller-dir}/HIROController.cpp
    ${hirocontroller-dir}/HIROControllerStub.cpp
    ${hirocontroller-dir}/HiroNXStub.cpp
	../GripperManipulation/RobotLocalFunctions.cpp
)

set(headers
    ${hirocontroller-dir}/HIROController.h
    ${hirocontroller-dir}/HIROController.hh
    ${hirocontroller-dir}/HIROControllerStub.h
    ${hirocontroller-dir}/HiroNX.hh
    ${hirocontroller-dir}/HiroNXStub.h
	../GripperManipulation/RobotLocalFunctions.h
  )

include_directories(${hirocontroller-dir} ${PYTHON_INCLUDE_PATH}
    /usr/include /usr/include/rtm/idl .
)
add_library(${libname} SHARED ${sources} ${headers} )

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

if(UNIX)

  target_link_libraries(${libname} CnoidBodyPlugin CnoidGraspPlugin  ${PYTHON_LIBRARIES} uuid dl pthread omniORB4 omnithread omniDynamic4 RTC coil)

  install(TARGETS ${libname}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug)

elseif(MSVC)

  source_group("Header Files" FILES ${headers})

  target_link_libraries(${libname} Excade${EXCADE_VERSION_SUFFIX}
  )

  set_target_properties(${libname} PROPERTIES COMPILE_FLAGS "/wd4250 /wd4275")

  install(TARGETS ${libname}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION lib CONFIGURATIONS Release Debug)

endif(UNIX)
