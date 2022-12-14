find_package(PkgConfig)

# choreonoid
pkg_check_modules(CNOID REQUIRED choreonoid)
link_directories(${CNOID_LIBRARY_DIRS})
execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=plugindir choreonoid
  OUTPUT_VARIABLE CNOID_PLUGIN_SUBDIR
  OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=sharedir choreonoid
  OUTPUT_VARIABLE CNOID_SHARE_SUBDIR
  OUTPUT_STRIP_TRAILING_WHITESPACE)

# openhrp3.1
if(NOT DEFINED OPENHRP3_1_PREFIX)
  pkg_check_modules(OPENHRP3_1 REQUIRED openhrp3.1)
endif()
message("-- OPENHRP3_1_PREFIX: ${OPENHRP3_1_PREFIX}")

# hrpsys-base
if(NOT DEFINED HRPSYSBASE_PREFIX)
  pkg_check_modules(HRPSYSBASE REQUIRED hrpsys-base)
endif()
message("-- HRPSYSBASE_PREFIX: ${HRPSYSBASE_PREFIX}")

add_subdirectory(project)
