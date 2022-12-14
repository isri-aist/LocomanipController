add_fsm_state(InitialState InitialState.cpp)
target_link_libraries(InitialState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_state(GuiManipState GuiManipState.cpp)
target_link_libraries(GuiManipState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_data_directory(data)
