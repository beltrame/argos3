# compile test loop functions (reuses the camera_sensor test controller)
add_library(scale_50_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_link_libraries(scale_50_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_entities
    argos3plugin_${ARGOS_BUILD_FOR}_footbot)
# configure experiment
set(CAMERA_TEST_CONTROLLER_LIB
  ${CMAKE_CURRENT_BINARY_DIR}/../camera_sensor/libcamera_test_controller)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration.argos)
# define test
add_test(
   NAME photorealism_scale_50
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration.argos")
set_tests_properties(photorealism_scale_50
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}"
             TIMEOUT 600)
