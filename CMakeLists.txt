cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME locomanip_controller)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/LocomanipController)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR ON)
option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

include(cmake/base.cmake)
project(locomanip_controller LANGUAGES CXX)

OPTION(ENABLE_CNOID "Install Choreonoid files" ON)

# mc_rtc
add_project_dependency(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required for this controller")
endif()

if(DEFINED CATKIN_DEVEL_PREFIX)
  set(DOXYGEN_HTML_OUTPUT html)
  if(ENABLE_CNOID)
    set(CNOID_ROS_UTILS cnoid_ros_utils)
  else()
    set(CNOID_ROS_UTILS "")
  endif()
  find_package(catkin REQUIRED COMPONENTS
    baseline_walking_controller
    ${CNOID_ROS_UTILS}
    )

  catkin_package(
    CATKIN_DEPENDS
    baseline_walking_controller
    DEPENDS EIGEN3
    INCLUDE_DIRS include
    LIBRARIES LocomanipController
    )

  include_directories(include ${catkin_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS})

  set(BWC_STATES_LIBRARIES "${baseline_walking_controller_DEVEL_PREFIX}/lib/baseline_walking_controller/states")
  set(BWC_STATES_FILES "${BWC_STATES_LIBRARIES}/data")
  set(STATES_INSTALL_PREFIX "${CATKIN_DEVEL_PREFIX}/lib")
  set(CONFIG_OUT "${CATKIN_DEVEL_PREFIX}/lib/mc_controller/etc/LocomanipController.yaml")
  set(CONFIG_DIR_STANDALONE "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/LocomanipController")
  set(CONFIG_DIR_CATKIN "${CATKIN_DEVEL_PREFIX}/lib/mc_controller/LocomanipController")
  if(EXISTS "${CONFIG_DIR_STANDALONE}" AND IS_DIRECTORY "${CONFIG_DIR_STANDALONE}")
    execute_process(COMMAND cmake -E create_symlink
      "${CONFIG_DIR_STANDALONE}" "${CONFIG_DIR_CATKIN}"
    )
  endif()

  configure_file(etc/mc_rtc.in.yaml ${PROJECT_SOURCE_DIR}/etc/mc_rtc.yaml @ONLY)
else()
  set(DOXYGEN_HTML_OUTPUT doxygen-html)
  set(STATES_INSTALL_PREFIX ${MC_CONTROLLER_INSTALL_PREFIX})
  set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/etc/LocomanipController.yaml")
  add_project_dependency(baseline_walking_controller REQUIRED)
  if(ENABLE_CNOID)
    add_project_dependency(CnoidRosUtils REQUIRED)
  endif()
endif()

configure_file(etc/LocomanipController.in.yaml ${CONFIG_OUT} @ONLY)
install(FILES ${CONFIG_OUT} DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc/)

add_subdirectory(src)

add_subdirectory(description)

if(ENABLE_CNOID)
  add_subdirectory(cnoid)
endif()

if(BUILD_TESTING OR CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()

set(PACKAGE_EXTRA_MACROS "set(LOCOMANIP_CONTROLLER_STATES_LIBRARIES \"${MC_CONTROLLER_INSTALL_PREFIX}/locomanip_controller/states\")
set(LOCOMANIP_CONTROLLER_STATES_FILES \"\${LOCOMANIP_CONTROLLER_STATES_LIBRARIES}/data\")")
