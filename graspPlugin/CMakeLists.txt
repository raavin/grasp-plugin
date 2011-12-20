if(ENABLE_INSTALL_RPATH)
  set(CMAKE_INSTALL_RPATH "$ORIGIN/..")
endif()

set(GRASP_PLUGINS "Grasp" CACHE STRING
  "Sub-directories containing plugins to compile in the each directory")

foreach(subdirectory ${GRASP_PLUGINS})
  add_subdirectory(${subdirectory})
endforeach()

add_subdirectory(RobotModels)


