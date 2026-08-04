# compile test loop functions (reuses the camera_sensor test controller)
add_library(lights_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_include_directories(lights_loop_functions
  PRIVATE ${FILAMENT_INCLUDE_DIRS})
target_link_libraries(lights_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_entities
    argos3plugin_${ARGOS_BUILD_FOR}_footbot
    argos3plugin_${ARGOS_BUILD_FOR}_photorealism)
set(CAMERA_TEST_CONTROLLER_LIB
  ${CMAKE_CURRENT_BINARY_DIR}/../camera_sensor/libcamera_test_controller)
set(LAMP_LUMENS "6000")

# Variant 1: the camera opened up for a lamp-lit scene. The box under
# the lamp must be clearly bright and the far one must stay black.
set(EXPOSURE_NODE "      <exposure aperture=\"2.8\" shutter_speed=\"0.0166\" sensitivity=\"400\" />")
set(MIN_NEAR_LUMINANCE "0.25")
set(MAX_NEAR_LUMINANCE "1.0")
set(MAX_FAR_LUMINANCE "0.02")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration.argos)

# Variant 2: the same lamp at the default sunny-16 exposure, which is
# several stops darker. Now neither box may register. This is what
# proves the <exposure> node reaches the cameras instead of being
# parsed and discarded, and it records that a lamp-lit scene is
# exposure-dependent, so a black render is diagnosed as an exposure
# that was never opened up rather than as a broken light.
set(EXPOSURE_NODE "      <!-- no <exposure>: the sunny-16 default -->")
set(MIN_NEAR_LUMINANCE "0.0")
set(MAX_NEAR_LUMINANCE "0.05")
set(MAX_FAR_LUMINANCE "0.02")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration_sunny.argos)

add_test(
   NAME photorealism_lights
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration.argos")
set_tests_properties(
  photorealism_lights
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")

add_test(
   NAME photorealism_exposure
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration_sunny.argos")
set_tests_properties(
  photorealism_exposure
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
