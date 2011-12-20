
# c) Tokuo Tsuji (Kyushu univ./AIST)


set(libname CnoidVisionTriggerPlugin)

set(rtc-dir ${CMAKE_CURRENT_SOURCE_DIR}/rtc/)

set(sources
	VisionTriggerPlugin.cpp
	VisionTriggerRtcController.cpp
	VisionTriggerBar.cpp
	${rtc-dir}RecognitionServiceSkel.cpp
	${rtc-dir}VisionRecognitionTrigger.cpp
)

set(headers
  )


add_library(${libname} SHARED ${sources} ${headers} )

add_definitions(-Wall -fPIC -O2 -I/usr/include -I/usr/include/rtm/idl -I.)
include_directories(${rtc-dir})

#set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0 LINK_FLAGS -export-dynamic -Wl,-z,defs -L/usr/lib)
set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

#add_definitions(-export-dynamic -Wl,-z,defs -luuid -ldl -L/usr/lib -lpthread -lomniORB4 -lomnithread -lomniDynamic4 -lRTC -lcoil)

#add_custom_command(
#	OUTPUT ${libname}
#	COMMAND g++ ${libname}   `rtm-config --libs`
#	DEPENDS ${sources}
#	)

target_link_libraries(${libname}  CnoidBodyPlugin CnoidGraspPlugin  uuid dl pthread omniORB4 omnithread omniDynamic4 RTC coil  )
#target_link_libraries(${libname} ${OPENRTM_LIBRARIES} Excade${EXCADE_VERSION_SUFFIX} ExcadeGrasplot${EXCADE_VERSION_SUFFIX}   )

  install(TARGETS ${libname}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug)
	
	
