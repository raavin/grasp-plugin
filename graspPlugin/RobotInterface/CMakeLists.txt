
# @author Shin'ichiro Nakaoka (AIST)

set(rtc-dir ./ArmControllerRtc)
set(libname CnoidRobotInterfacePlugin)

set(sources
	RobotInterfacePlugin.cpp
	RobotInterface.cpp
	RobotInterfaceBar.cpp
	ArmControllerRtc.cpp

    ${rtc-dir}/ArmController.cpp
    ${rtc-dir}/HIROControllerStub.cpp
    ${rtc-dir}/HiroNXStub.cpp
    ${rtc-dir}/ManipulatorCommonInterface_CommonStub.cpp
    ${rtc-dir}/ManipulatorCommonInterface_MiddleLevelStub.cpp
    ${rtc-dir}/ManipulatorCommonInterface_DataTypesSK.cc

	../GripperManipulation/RobotLocalFunctions.cpp
)

set(headers
	ArmControllerRtc.h
	RobotInterface.h
	RobotInterfaceBar.h

    ${rtc-dir}/ArmController.h
    ${rtc-dir}/HIROController.hh
    ${rtc-dir}/HIROControllerStub.h
    ${rtc-dir}/HiroNX.hh
    ${rtc-dir}/HiroNXStub.h
    ${rtc-dir}/ManipulatorCommonInterface_Common.hh
    ${rtc-dir}/ManipulatorCommonInterface_CommonStub.h
    ${rtc-dir}/ManipulatorCommonInterface_MiddleLevel.hh
    ${rtc-dir}/ManipulatorCommonInterface_MiddleLevelStub.h
	../GripperManipulation/RobotLocalFunctions.h
  )

include_directories(${rtc-dir} ${PYTHON_INCLUDE_PATH}
    /usr/include /usr/include/rtm/idl .
)
add_library(${libname} SHARED ${sources} ${headers} )

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

if(UNIX)

  target_link_libraries(${libname} 
  	CnoidBodyPlugin CnoidGraspPlugin  
  	${PYTHON_LIBRARIES} uuid dl pthread 
  	omniORB4 omnithread omniDynamic4 RTC coil)

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
