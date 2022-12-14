set(LMC_CNOID_CONFIG_FILE_LIST
  rtc.conf.choreonoid
  robot.conf
  RobotHardware.conf
  PDcontroller.conf.choreonoid
  PDgains_sim.dat
)
foreach(CNOID_CONFIG_FILE IN LISTS LMC_CNOID_CONFIG_FILE_LIST)
  execute_process(COMMAND cmake -E create_symlink
    "${HRPSYSBASE_PREFIX}/share/hrpsys/samples/JVRC1/${CNOID_CONFIG_FILE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/${CNOID_CONFIG_FILE}"
  )
endforeach()

set(LMC_PROJECT_FILE_LIST
  LMC_JVRC1_Cart.in.cnoid
  )
foreach(PROJECT_FILE_IN IN LISTS LMC_PROJECT_FILE_LIST)
  string(REPLACE ".in." "." PROJECT_FILE_OUT "${PROJECT_FILE_IN}")
  configure_file(${PROJECT_FILE_IN}
    ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_FILE_OUT}
    @ONLY)
endforeach()
