# compile test loop functions (they drive the randomization API)
add_library(randomization_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_include_directories(randomization_loop_functions
  PRIVATE ${FILAMENT_INCLUDE_DIRS})
target_link_libraries(randomization_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_entities
    argos3plugin_${ARGOS_BUILD_FOR}_photorealism)
# three variants: same seed twice, then a different seed
set(CAMERA_TEST_CONTROLLER_LIB
  ${CMAKE_CURRENT_BINARY_DIR}/../camera_sensor/libcamera_test_controller)
set(TEST_SEED 42)
set(TEST_DUMP_PREFIX r1)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration_r1.argos)
set(TEST_DUMP_PREFIX r2)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration_r2.argos)
set(TEST_SEED 43)
set(TEST_DUMP_PREFIX r3)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration_r3.argos)
# same seed -> identical RGB frames; different seed -> different RGB
# but identical depth and segmentation
add_test(
   NAME photorealism_randomization
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "\
env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration_r1.argos && \
env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration_r2.argos && \
env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration_r3.argos && \
cmp r1_rgb.ppm r2_rgb.ppm && \
! cmp -s r1_rgb.ppm r3_rgb.ppm && \
cmp r1_depth.pgm r3_depth.pgm && \
cmp r1_seg.pgm r3_seg.pgm")
set_tests_properties(photorealism_randomization
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
