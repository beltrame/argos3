add_library(drone_fast_jolt_loop_functions MODULE loop_functions.h loop_functions.cpp)
target_link_libraries(drone_fast_jolt_loop_functions
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_entities
  argos3plugin_${ARGOS_BUILD_FOR}_drone)
add_dependencies(drone_fast_jolt_loop_functions drone_mesh_jolt_controller)
set(DRONE_MESH_CONTROLLER ${CMAKE_BINARY_DIR}/testing/jolt/drone_mesh/libdrone_mesh_jolt_controller)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration.argos)
add_test(
  NAME jolt_drone_fast
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration.argos)
set_tests_properties(jolt_drone_fast
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
