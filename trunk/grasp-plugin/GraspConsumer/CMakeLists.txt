
# @author Tokuo Tsuji, Shin'ichiro Nakaoka (AIST)



set(libname CnoidGraspRtcPlugin)

set(rtc-dir ${CMAKE_CURRENT_SOURCE_DIR}/rtc/)

set(sources
GraspRtcPlugin.cpp
GraspRtcController.cpp
GraspRtcBar.cpp
GraspControllerSVC_impl.cpp
${rtc-dir}GraspControllerSkel.cpp
${rtc-dir}GraspConsumer.cpp 
)

set(headers
GraspConsumer.h
  )


add_library(${libname} SHARED ${sources} ${headers} )

add_definitions(-Wall -fPIC -O2 -I/usr/include -I/usr/include/rtm/idl -I.)
include_directories(${rtc-dir})

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

target_link_libraries(${libname} CnoidBodyPlugin CnoidGraspPlugin uuid dl pthread omniORB4 omnithread omniDynamic4 RTC coil  )

  install(TARGETS ${libname}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug)
	
	
