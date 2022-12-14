set(CONTROLLER_NAME LocomanipController)

add_library(${CONTROLLER_NAME} SHARED
  LocomanipController.cpp
  HandTypes.cpp
  ManipPhase.cpp
  ManipManager.cpp
  State.cpp
  )
target_link_libraries(${CONTROLLER_NAME} PUBLIC
  mc_rtc::mc_rtc_utils
  mc_rtc::mc_rbdyn
  mc_rtc::mc_control_fsm
  mc_rtc::mc_rtc_ros
  ${catkin_LIBRARIES}
  )

add_controller(${CONTROLLER_NAME}_controller lib.cpp "")
set_target_properties(${CONTROLLER_NAME}_controller PROPERTIES OUTPUT_NAME "${CONTROLLER_NAME}")
target_link_libraries(${CONTROLLER_NAME}_controller PUBLIC ${CONTROLLER_NAME})

add_subdirectory(states)
