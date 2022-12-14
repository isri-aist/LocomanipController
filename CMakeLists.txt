cmake_minimum_required(VERSION 3.1)
project(locomanip_controller)

# If CMAKE_CXX_STANDARD is not specified, the features specified in target_compile_features or INTERFACE_COMPILE_FEATURES
# may cause -std=gnu++11 to be added to the compile options and overwrite the C++ version specified in add_compile_options.
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  baseline_walking_controller
  )

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# mc_rtc
find_package(mc_rtc REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  baseline_walking_controller
  DEPENDS EIGEN3
  INCLUDE_DIRS include
  LIBRARIES LocomanipController
  )

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

configure_file(etc/LocomanipController.in.yaml
  "${CATKIN_DEVEL_PREFIX}/lib/mc_controller/etc/LocomanipController.yaml" @ONLY)

configure_file(etc/mc_rtc.in.yaml ${PROJECT_SOURCE_DIR}/etc/mc_rtc.yaml @ONLY)

add_subdirectory(src)

OPTION(ENABLE_CNOID "Install Choreonoid files" ON)
if(ENABLE_CNOID)
  add_subdirectory(cnoid)
endif()

OPTION(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)
if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()
