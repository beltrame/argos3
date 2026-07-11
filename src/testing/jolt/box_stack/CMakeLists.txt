# compile test loop functions
add_library(box_stack_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_link_libraries(box_stack_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_entities)
# configure two identical experiments writing different pose files
set(POSES_FILE poses_1.txt)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration_1.argos)
set(POSES_FILE poses_2.txt)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration_2.argos)
# settling test
add_test(
   NAME jolt_box_stack
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration_1.argos)
# bitwise determinism: two runs must produce identical final poses
add_test(
   NAME jolt_determinism
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "${CMAKE_BINARY_DIR}/core/argos3 -zc configuration_1.argos && ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration_2.argos && cmp poses_1.txt poses_2.txt")
set_tests_properties(jolt_box_stack jolt_determinism
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
