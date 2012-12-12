cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()
rosbuild_genmsg()
rosbuild_add_boost_directories()

#...: dynamic reconfigure :.....................................................

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#...: compiler options :........................................................

#...: gnu++0x
if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        add_definitions(-std=gnu++0x)
    else(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        message(SEND_ERROR "You need GCC version 4.6 or greater to compile this package.")
    endif(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
endif(CMAKE_COMPILER_IS_GNUCXX)

#...: enable all but certain warnings
add_definitions(-Wall)
add_definitions(-Wno-unknown-pragmas)
add_definitions(-Wno-deprecated)
add_definitions(-fpermissive)

#...: default paths  :..........................................................

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/include)

#...: gsl :.....................................................................

find_package(PkgConfig REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)

#...: target libraries :........................................................

#...: pmd_tools_nodelet
rosbuild_add_library(pmd_tools_nodelet
  src/saturation_filter_nodelet.cpp
  src/image_histogram_nodelet.cpp
)
rosbuild_link_boost(pmd_tools_nodelet
  system
  signals
)
target_link_libraries(pmd_tools_nodelet
  ${gsl_LIBRARIES}
)

#...: target executables :......................................................

#...: image_histogram_node
rosbuild_add_executable(image_histogram_node
  src/image_histogram_node.cpp
)

#...: target executables :......................................................

#...: plane_fit
#rosbuild_add_executable(plane_fit
  #src/plane_fit.cpp
#)

#...: saturation_filter
#rosbuild_add_executable(saturation_filter
  #nodes/saturation_filter.cpp
#)
#rosbuild_link_boost(saturation_filter
  #system
  #signals
#)
