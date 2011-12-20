cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#set(ROS_BUILD_STATIC_LIBS true)

#set(libname CnoidRosSamplePlugin)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(talker src/talker.cpp)
rosbuild_add_executable(listener listener.cpp)

#rosbuild_add_executable(add_two_ints_server src/add_two_ints_server.cpp)
#rosbuild_add_executable(add_two_ints_client src/add_two_ints_client.cpp)


#rosbuild_add_executable(talker src/talker.cpp)


#find_package(Choreonoid)
#include_directories(${Choreonoid_INCLUDE_DIRS})
#link_directories(${Choreonoid_LIBRARY_DIRS})

# qt
find_package(Qt4 REQUIRED)
set(QT_USE_QTOPENGL TRUE)
#set(QT_USE_QTTEST TRUE)
include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR} ${QT_OPENGL_INCLUDE_DIR})
add_definitions(-DQT_NO_KEYWORDS)


set(LIBRARY_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../lib/choreonoid-1.0")
set(libname CnoidHelloWorld2Plugin)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../../../include" "${CMAKE_CURRENT_SOURCE_DIR}/../../../..")





  set(target CnoidHelloWorld2Plugin)
  add_library(${target} SHARED HelloWorldPlugin.cpp src/talker.cpp)
  target_link_libraries(${target} CnoidGuiBase ros)
#  apply_common_setting_for_plugin(${target})
