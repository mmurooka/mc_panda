set(customizer PandaCustomizer)
set(src PandaCustomizer.cpp)

execute_process(
  COMMAND pkg-config --variable=prefix openhrp3.1
  OUTPUT_VARIABLE OPENHRP_DIR
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
message(WARNING "OPENHRP_DIR=${OPENHRP_DIR}")
set(CMAKE_INSTALL_PREFIX ${OPENHRP_DIR})

execute_process(
  COMMAND pkg-config --cflags-only-I openhrp3.1
  OUTPUT_VARIABLE OPENHRP_INCLUDE_DIRS
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(
  COMMAND pkg-config --libs openhrp3.1
  OUTPUT_VARIABLE OPENHRP_LIBRARIES
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(
  COMMAND pkg-config choreonoid --variable=plugindir
  OUTPUT_VARIABLE CNOID_PLUGIN_DIR
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)

if(RESULT EQUAL 0)
  string(REGEX REPLACE "-I" ";" OPENHRP_INCLUDE_DIRS ${OPENHRP_INCLUDE_DIRS})
endif()

include_directories(
  ${OPENHRP_INCLUDE_DIRS}
  ${OPENHRP_DIR}/include/OpenHRP-3.1/hrpModel
  )
add_library(${customizer} SHARED ${src})
set_target_properties(${customizer} PROPERTIES PREFIX "")

install(TARGETS ${customizer}
  DESTINATION ${OPENHRP_DIR}/share/OpenHRP-3.1/customizer)
install(TARGETS ${customizer}
  DESTINATION ${CNOID_PLUGIN_DIR}/customizer)
