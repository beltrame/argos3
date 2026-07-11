# compile test loop functions (reuses the camera_sensor test controller)
add_library(robot_visuals_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_include_directories(robot_visuals_loop_functions
  PRIVATE ${FILAMENT_INCLUDE_DIRS})
target_link_libraries(robot_visuals_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_entities
    argos3plugin_${ARGOS_BUILD_FOR}_footbot
    argos3plugin_${ARGOS_BUILD_FOR}_photorealism)
# configure experiment
set(CAMERA_TEST_CONTROLLER_LIB
  ${CMAKE_CURRENT_BINARY_DIR}/../camera_sensor/libcamera_test_controller)
set(ROBOT_VISUALS_DUMP "")
set(ROBOT_VISUALS_VISUALIZATION "<!-- No visualization: this test must run fully headless -->")
set(ROBOT_VISUALS_ASSET_PATH "")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration.argos)
set(ROBOT_VISUALS_DUMP "view")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration_dump.argos)
# glTF variant: same assertions, robot rendered from the asset registry
set(ROBOT_VISUALS_ASSET_PATH "asset_path=\"${CMAKE_SOURCE_DIR}/plugins/simulator/photorealism/assets\"")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/configuration_gltf.argos)
set(ROBOT_VISUALS_ASSET_PATH "")
# define test
add_test(
   NAME photorealism_robot_visuals
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration.argos")
set_tests_properties(photorealism_robot_visuals
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
add_test(
   NAME photorealism_robot_visuals_gltf
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "env -u DISPLAY -u WAYLAND_DISPLAY ${CMAKE_BINARY_DIR}/core/argos3 -zc configuration_gltf.argos")
set_tests_properties(photorealism_robot_visuals_gltf
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")

# viewer variant: runs only when a display is available (the window
# opens briefly and closes by itself)
if(TARGET argos3plugin_${ARGOS_BUILD_FOR}_filament)
  set(ROBOT_VISUALS_DUMP "")
  set(ROBOT_VISUALS_VISUALIZATION "<visualization><filament autoclose=\"30\" speed=\"0\" /></visualization>")
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
    ${CMAKE_CURRENT_BINARY_DIR}/configuration_viewer.argos)
  add_test(
     NAME photorealism_viewer
     WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
     COMMAND bash -c "if [ -z \"$DISPLAY\" ]; then exit 77; fi; ${CMAKE_BINARY_DIR}/core/argos3 -c configuration_viewer.argos")
  set_tests_properties(photorealism_viewer
    PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}"
               SKIP_RETURN_CODE 77)
endif()
