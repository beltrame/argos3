# compile test loop functions (reuses the camera_sensor test controller)
add_library(scenery_depth_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_include_directories(scenery_depth_loop_functions
  PRIVATE ${FILAMENT_INCLUDE_DIRS})
target_link_libraries(scenery_depth_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_entities
    argos3plugin_${ARGOS_BUILD_FOR}_footbot
    argos3plugin_${ARGOS_BUILD_FOR}_photorealism)
# configure experiment
set(CAMERA_TEST_CONTROLLER_LIB
  ${CMAKE_CURRENT_BINARY_DIR}/../camera_sensor/libcamera_test_controller)
set(SCENERY_MODEL
  ${CMAKE_SOURCE_DIR}/plugins/simulator/photorealism/assets/foot-bot.glb)
# Nearest surface of the scaled prop, measured from the camera; the
# tolerance is wide because the asset is a rounded body, but far below
# the 20 m far plane the broken code reported.
set(SCENERY_EXPECTED_DEPTH "0.90")
set(SCENERY_DEPTH_TOLERANCE "0.40")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration.argos)
# define test
add_test(
   NAME photorealism_scenery_depth
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration.argos")
set_tests_properties(
  photorealism_scenery_depth
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
