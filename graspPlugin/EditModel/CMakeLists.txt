
# @author Shin'ichiro Nakaoka (AIST)

set(libname CnoidEditModelPlugin)

set(sources
  EditModelPlugin.cpp
  )

set(headers
  )

add_library(${libname} SHARED ${sources} ${headers})

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

if(UNIX)

  target_link_libraries(${libname} CnoidBodyPlugin)

elseif(MSVC)

  source_group("Header Files" FILES ${headers})
  target_link_libraries(${libname} CnoidGuiBase CnoidBodyPlugin)
  set_target_properties(${libname} PROPERTIES COMPILE_FLAGS "/wd4250 /wd4275")

endif(UNIX)
