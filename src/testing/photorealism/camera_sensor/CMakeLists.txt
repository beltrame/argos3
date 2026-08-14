# compile test controller
add_library(camera_test_controller MODULE
  controller.h
  controller.cpp)
target_link_libraries(camera_test_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)
# compile test loop functions (they inspect the photorealism medium)
add_library(camera_test_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_include_directories(camera_test_loop_functions
  PRIVATE ${FILAMENT_INCLUDE_DIRS})
target_link_libraries(camera_test_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_entities
    argos3plugin_${ARGOS_BUILD_FOR}_footbot
    argos3plugin_${ARGOS_BUILD_FOR}_photorealism)
# configure the two experiment variants
set(TEST_LATENCY_MODE "pipelined")
set(TEST_EXPECTED_LATENCY "1")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration_pipelined.argos)
set(TEST_LATENCY_MODE "immediate")
set(TEST_EXPECTED_LATENCY "0")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration_immediate.argos)
# multi-mount variant: three cameras on one sensor
set(TEST_LATENCY_MODE "immediate")
set(TEST_EXPECTED_LATENCY "0")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration_multi.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration_multi.argos)
# define tests
add_test(
   NAME photorealism_camera_sensor_pipelined
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration_pipelined.argos")
add_test(
   NAME photorealism_camera_sensor_immediate
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration_immediate.argos")
add_test(
   NAME photorealism_camera_sensor_multi
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration_multi.argos")
set_tests_properties(
  photorealism_camera_sensor_pipelined
  photorealism_camera_sensor_immediate
  photorealism_camera_sensor_multi
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
