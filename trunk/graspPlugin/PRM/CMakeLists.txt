# c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)

set(libname CnoidPRMPlugin)

set(sources
  SBLinterface.cpp
  mpkConfigChecker.cpp
  mpkRobots.cpp
  mpkSimpleSegmentChecker.cpp
  sblPlanner.cpp
  sblRn.cpp
  sblTree.cpp
  sblRandVal.cpp
  mpkPathSmoother.cpp
#  tsp.cpp
#  tspCity.cpp
#  tspTour.cpp
#  tspVectornd.cpp
  TrajectoryPlanner.cpp
  TrajectoryBar.cpp
  TrajectoryPlugin.cpp
)

set(headers
  )

#add_definitions(-DDO_TOLERANCE_TEST)

add_library(${libname} SHARED ${sources} ${headers} )
set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

if(UNIX)

  target_link_libraries(${libname} CnoidBodyPlugin CnoidPoseSeqPlugin CnoidGraspPlugin)

  install(TARGETS ${libname}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug)

elseif(MSVC)

  source_group("Header Files" FILES ${headers})

  set_target_properties(${libname} PROPERTIES LINK_FLAGS /NODEFAULTLIB:LIBCMT )
  set_target_properties(${libname} PROPERTIES DEBUG_POSTFIX d )
  set_target_properties(${libname} PROPERTIES COMPILE_FLAGS "/wd4250 /wd4275")

  install(TARGETS ${libname}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION lib CONFIGURATIONS Release Debug)
    

endif(UNIX)
