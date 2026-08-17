# compile test loop functions (reuses the camera_sensor test controller)
add_library(debug_overlay_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_include_directories(debug_overlay_loop_functions
  PRIVATE ${FILAMENT_INCLUDE_DIRS})
target_link_libraries(debug_overlay_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_entities
    argos3plugin_${ARGOS_BUILD_FOR}_footbot
    argos3plugin_${ARGOS_BUILD_FOR}_photorealism)
# configure experiment
set(CAMERA_TEST_CONTROLLER_LIB
  ${CMAKE_CURRENT_BINARY_DIR}/../camera_sensor/libcamera_test_controller)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration.argos)
# define test
add_test(
   NAME photorealism_debug_overlay
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration.argos")
set_tests_properties(
  photorealism_debug_overlay
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
