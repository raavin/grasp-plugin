# c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)

set(libname CnoidGraspPlugin)

set(sources
  GraspPlugin.cpp
  PlanBase.cpp
  PlanInterface.cpp
  GraspController.cpp
  PlaceController.cpp
  Finger.cpp
  Arm.cpp
  readtext.cpp
  ConvexAnalysis.cpp 
  VectorMath.cpp
  ForceClosureTest.cpp
  GraspBar.cpp
  GraspPluginManager.cpp
)

set(headers
	Arm.h
	Finger.h
	ConvexAnalysis.h
	ForceClosureTest.h
	GraspBar.h
	PlanBase.h
    PlanInterface.h
	GraspController.h
    PlaceController.h
	GraspSceneBody.h
	VectorMath.h
	readtext.h
	GraspPluginManager.h
	exportdef.h
)

add_library(${libname} SHARED ${sources} ${headers} )
add_definitions(-g)
set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)
#set_target_properties(${libname} PROPERTIES DEFINE_SYMBOL "EXCADE_OPENHRP_MAKE_DLL")
include_directories(${PYTHON_INCLUDE_PATH})

if(UNIX)

	target_link_libraries(${libname} CnoidBodyPlugin qhull ${PYTHON_LIBRARIES} boost_regex boost_python)


  install(TARGETS ${libname}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug)

elseif(MSVC)

  source_group("Header Files" FILES ${headers})

  target_link_libraries(${libname}
    Excade${EXCADE_VERSION_SUFFIX}
#    optimized hrpUtil-${OPENHRP_VERSION} debug hrpUtil-${OPENHRP_VERSION}d
    optimized hrpModel-${OPENHRP_VERSION} debug hrpModel-${OPENHRP_VERSION}d
    optimized hrpCollision-${OPENHRP_VERSION} debug hrpCollision-${OPENHRP_VERSION}d
    optimized hrpCorbaStubSkel-${OPENHRP_VERSION} debug hrpCorbaStubSkel-${OPENHRP_VERSION}d
    ${LAPACK_LIBRARIES}
	qhull
	ExcadeRobotics${EXCADE_VERSION_SUFFIX}
  )

  set_target_properties(${libname} PROPERTIES LINK_FLAGS /NODEFAULTLIB:LIBCMT )
  set_target_properties(${libname} PROPERTIES DEBUG_POSTFIX d )
  set_target_properties(${libname} PROPERTIES COMPILE_FLAGS "/wd4250 /wd4275")

  if(INSTALL_SDK)
    install(TARGETS ${libname}
      RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
      LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
      ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug)
  else()
    install(TARGETS ${libname}
      RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
      LIBRARY DESTINATION lib CONFIGURATIONS Release Debug)
  endif()
    

endif(UNIX)


if(ENABLE_INSTALL_RPATH)
  SET(CMAKE_INSTALL_RPATH "$ORIGIN/..")
endif()

if(UNIX)
  if(CMAKE_MAJOR_VERSION EQUAL 2 AND CMAKE_MINOR_VERSION LESS 5)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/${RELATIVE_PLUGINS_INSTALL_PATH})
  else()
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${RELATIVE_PLUGINS_INSTALL_PATH})
  endif()
endif()


#add_subdirectory(HRG/HRP3P/GrasplotPlugin)

# check qhull
#find_package(qhull REQUIRED)

#include_directories(${QHULL_INCLUDE_DIRS})
#link_directories(${QHULL_LIBRARY_DIRS})

if(INSTALL_SDK)
  install(FILES ${headers} DESTINATION ${CNOID_HEADER_SUBDIR}/extplugin/graspPlugin/Grasp)
endif()

    

