# compile test controller
add_library(external_estimator_test_controller MODULE
  controller.h
  controller.cpp)
target_link_libraries(external_estimator_test_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)
# compile test loop functions
add_library(external_estimator_test_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_link_libraries(external_estimator_test_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_entities
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
    argos3plugin_${ARGOS_BUILD_FOR}_footbot)
# the harness that runs the stub estimator alongside argos3
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/run_test.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/run_test.sh
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

#
# Two tests over the same scenario:
#   protocol - the wire contract and the one-tick round trip
#   wheels   - the wheel-encoder dead reckoning, checked against ground
#              truth by having the stub echo it back
#
set(TEST_ALIGNMENT "none")
set(TEST_FIDUCIAL "")
foreach(_mode protocol wheels)
  set(TEST_MODE ${_mode})
  set(TEST_SOCKET ${CMAKE_CURRENT_BINARY_DIR}/estimator_${_mode}.sock)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
    ${CMAKE_CURRENT_BINARY_DIR}/${_mode}.argos)
endforeach()

#
# Frame alignment. The stub echoes the wheel-encoder pose, which lives
# in the robot's own start frame, and the medium has to place it in the
# world. The loop functions then compare against ground truth with NO
# manual offset, so a transposed, inverted or wrongly-anchored
# composition fails immediately.
#
#   ground_truth  places the robot from its own ground truth
#   fiducial      places it from a surveyed marker it must actually
#                 look at, with the noise set to zero so the expected
#                 answer stays exact. The foot-bot starts at (-0.5,0,0)
#                 facing +x, so a marker at (1.5,0,0.2) is dead ahead
#                 and inside both the range and the field of view.
#
set(TEST_MODE "aligned")
set(TEST_ALIGNMENT "ground_truth")
set(TEST_SOCKET ${CMAKE_CURRENT_BINARY_DIR}/estimator_align_gt.sock)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/align_gt.argos)

set(TEST_ALIGNMENT "fiducial")
set(TEST_FIDUCIAL "      <fiducial id=\"m0\" position=\"1.5,0,0.2\" />")
set(TEST_SOCKET ${CMAKE_CURRENT_BINARY_DIR}/estimator_align_fid.sock)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/align_fid.argos)
set(TEST_FIDUCIAL "")

add_test(
   NAME external_estimator_protocol
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_CURRENT_BINARY_DIR}/run_test.sh
           ${CMAKE_CURRENT_BINARY_DIR}/protocol.argos
           ${CMAKE_CURRENT_BINARY_DIR}/estimator_protocol.sock)
set_tests_properties(external_estimator_protocol
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")

add_test(
   NAME external_estimator_wheels
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_CURRENT_BINARY_DIR}/run_test.sh
           ${CMAKE_CURRENT_BINARY_DIR}/wheels.argos
           ${CMAKE_CURRENT_BINARY_DIR}/estimator_wheels.sock
           --echo-wheels)
set_tests_properties(external_estimator_wheels
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")

add_test(
   NAME external_estimator_align_ground_truth
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_CURRENT_BINARY_DIR}/run_test.sh
           ${CMAKE_CURRENT_BINARY_DIR}/align_gt.argos
           ${CMAKE_CURRENT_BINARY_DIR}/estimator_align_gt.sock
           --echo-wheels)
set_tests_properties(external_estimator_align_ground_truth
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")

add_test(
   NAME external_estimator_align_fiducial
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_CURRENT_BINARY_DIR}/run_test.sh
           ${CMAKE_CURRENT_BINARY_DIR}/align_fid.argos
           ${CMAKE_CURRENT_BINARY_DIR}/estimator_align_fid.sock
           --echo-wheels)
set_tests_properties(external_estimator_align_fiducial
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
