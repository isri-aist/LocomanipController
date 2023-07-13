set(MODEL_FILE_LIST
  texture/CheckeredPattern.png
  Cart.body
  CartHighFriction.body
  FloorCheckeredPattern.body
  LMC_materials.yaml
)

foreach(MODEL_FILE ${MODEL_FILE_LIST})
  configure_file(${MODEL_FILE} ${CMAKE_CURRENT_BINARY_DIR}/model/environment/LmcCnoid/${MODEL_FILE} COPYONLY)
endforeach()

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/model/environment/LmcCnoid DESTINATION ${CNOID_SHARE_SUBDIR}/model/environment)
