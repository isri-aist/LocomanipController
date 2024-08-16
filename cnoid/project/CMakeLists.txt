if(NOT ${CHOREONOID_USE_PYTHON2})
  execute_process(
    COMMAND python3 -c "import sysconfig; print(sysconfig.get_config_var(\"VERSION\"))"
    OUTPUT_VARIABLE PYTHON_VERSION
    RESULT_VARIABLE PYTHON_VERSION_SUCCESS
    OUTPUT_STRIP_TRAILING_WHITESPACE)
else()
  execute_process(
    COMMAND python2 -c "from distutils import sysconfig; print(sysconfig.get_config_var(\"VERSION\"))"
    OUTPUT_VARIABLE PYTHON_VERSION
    RESULT_VARIABLE PYTHON_VERSION_SUCCESS
    OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
if(NOT PYTHON_VERSION_SUCCESS STREQUAL 0)
  execute_process(
    COMMAND python -c "from distutils import sysconfig; print(sysconfig.get_config_var(\"VERSION\"))"
    OUTPUT_VARIABLE PYTHON_VERSION
    RESULT_VARIABLE PYTHON_VERSION_SUCCESS
    OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
message("-- Python version detected: ${PYTHON_VERSION}")

set(LMC_CNOID_CONFIG_FILE_LIST
  rtc.conf.choreonoid
  robot.conf
  RobotHardware.conf
  PDcontroller.conf.choreonoid
  PDgains_sim.dat
)
if(DEFINED CATKIN_DEVEL_PREFIX)
  foreach(CNOID_CONFIG_FILE IN LISTS LMC_CNOID_CONFIG_FILE_LIST)
    execute_process(COMMAND cmake -E create_symlink
      "${HRPSYSBASE_PREFIX}/share/hrpsys/samples/JVRC1/${CNOID_CONFIG_FILE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/${CNOID_CONFIG_FILE}"
    )
  endforeach()
endif()

file(GLOB LMC_PROJECT_FILE_LIST ${CMAKE_CURRENT_SOURCE_DIR}/*.in.cnoid)

if(DEFINED CATKIN_DEVEL_PREFIX)
  set(LMC_MODEL_DIR ${PROJECT_SOURCE_DIR}/cnoid/model)
  set(CNOID_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
  foreach(PROJECT_FILE_IN IN LISTS LMC_PROJECT_FILE_LIST)
    get_filename_component(PROJECT_FILE_IN ${PROJECT_FILE_IN} NAME)
    string(REPLACE ".in." "." PROJECT_FILE_OUT "${PROJECT_FILE_IN}")
    configure_file(${PROJECT_FILE_IN}
      ${CNOID_PROJECT_DIR}/${PROJECT_FILE_OUT}
      @ONLY)
  endforeach()
endif()

set(CNOID_PROJECT_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(LMC_MODEL_DIR ${CNOID_SHARE_SUBDIR}/model/environment/LmcCnoid)
foreach(PROJECT_FILE_IN IN LISTS LMC_PROJECT_FILE_LIST)
  get_filename_component(PROJECT_FILE_IN ${PROJECT_FILE_IN} NAME)
  string(REPLACE ".in." "." PROJECT_FILE_OUT "${PROJECT_FILE_IN}")
  configure_file(${PROJECT_FILE_IN}
    ${CNOID_PROJECT_DIR}/${PROJECT_FILE_OUT}
    @ONLY)
  install(FILES ${CNOID_PROJECT_DIR}/${PROJECT_FILE_OUT} DESTINATION ${HRPSYSBASE_PREFIX}/share/hrpsys/samples/JVRC1/)
endforeach()
