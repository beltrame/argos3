# compile test controller
add_library(lidar_test_controller MODULE
  controller.h
  controller.cpp)
target_link_libraries(lidar_test_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)
# configure the experiment: exact ranges
set(TEST_LATENCY_MODE "immediate")
set(TEST_MAX_ERROR "0.15")
set(TEST_MEAN_ERROR "0.02")
set(TEST_RANGE_NOISE "0")
set(TEST_MIN_MEAN_ERROR "0")
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

#
# The same room with 5 cm range noise. The mean absolute error of
# Gaussian noise is sigma*sqrt(2/pi) = 0.8*sigma, so ~0.04 m; the bounds
# below bracket that. The LOWER bound is the point of this test: an
# implementation that parses range_noise_std_dev and never applies it
# would sail through every other check in the suite.
#
set(TEST_MAX_ERROR "0.45")
set(TEST_MEAN_ERROR "0.08")
set(TEST_RANGE_NOISE "0.05")
set(TEST_MIN_MEAN_ERROR "0.02")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration_noise.argos)
add_test(
   NAME photorealism_lidar_range_noise
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration_noise.argos")
set_tests_properties(
  photorealism_lidar_range_noise
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
