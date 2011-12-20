
# @author Shin'ichiro Nakaoka


set(libname CnoidImageWidgetPlugin)


set(sources
  ImageWidgetPlugin.cpp
  ImageWidgetView.cpp
)


add_library(${libname} SHARED ${sources} ${headers} )

#add_definitions(-Wall -fPIC -O2 -I/usr/include -I/usr/include/rtm/idl -I.)
#include_directories(${rtc-dir})

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

target_link_libraries(${libname} CnoidBodyPlugin  )

  install(TARGETS ${libname}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug)
