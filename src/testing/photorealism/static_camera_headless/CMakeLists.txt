# compile test loop functions
add_library(photorealism_test_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_link_libraries(photorealism_test_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_entities)
# configure experiment
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration.argos)
# define test: runs with no display and checks that all frames were written
add_test(
   NAME photorealism_static_camera_headless
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "rm -rf frames && env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration.argos && test $(ls frames/frame_*.png | wc -l) -eq 10")
set_tests_properties(photorealism_static_camera_headless
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
