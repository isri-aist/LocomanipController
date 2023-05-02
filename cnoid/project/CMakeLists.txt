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
