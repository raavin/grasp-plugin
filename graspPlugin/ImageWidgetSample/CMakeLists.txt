
# @author Shin'ichiro Nakaoka


set(libname CnoidImageWidgetPlugin)


set(sources
  ImageWidgetPlugin.cpp
  ImageWidgetView.cpp
)

if(CNOID_VERSION EQUAL 1.0 OR CNOID_VERSION EQUAL 1.1)
 add_library(${libname} SHARED ${sources} ${headers})
else()
 add_cnoid_plugin(${libname} SHARED ${sources} ${headers})
endif()


#add_definitions(-Wall -fPIC -O2 -I/usr/include -I/usr/include/rtm/idl -I.)
#include_directories(${rtc-dir})

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

target_link_libraries(${libname} CnoidBodyPlugin  )

apply_common_setting_for_plugin(${libname})