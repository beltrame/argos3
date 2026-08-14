# compile test controller
add_library(lidar_test_controller MODULE
  controller.h
  controller.cpp)
target_link_libraries(lidar_test_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)
# configure the experiment
set(TEST_LATENCY_MODE "immediate")
set(TEST_MAX_ERROR "0.15")
set(TEST_MEAN_ERROR "0.02")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration.argos)
# define test
add_test(
   NAME photorealism_lidar_sensor
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration.argos")
set_tests_properties(
  photorealism_lidar_sensor
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
