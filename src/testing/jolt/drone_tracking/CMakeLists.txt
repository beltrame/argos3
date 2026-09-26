add_library(drone_tracking_jolt_loop_functions MODULE loop_functions.cpp)
target_link_libraries(drone_tracking_jolt_loop_functions
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_drone)
add_dependencies(drone_tracking_jolt_loop_functions drone_mesh_jolt_controller)
set(DRONE_MESH_CONTROLLER ${CMAKE_BINARY_DIR}/testing/jolt/drone_mesh/libdrone_mesh_jolt_controller)
foreach(AXIS x y)
  set(WAYPOINTS "0:0,0,1")
  foreach(TICK RANGE 300 1500 10)
    math(EXPR POSITION_TENTHS "(${TICK} - 300) * 3 / 10 + 30")
    math(EXPR WHOLE "${POSITION_TENTHS} / 10")
    math(EXPR FRACTION "${POSITION_TENTHS} % 10")
    if(AXIS STREQUAL "x")
      string(APPEND WAYPOINTS ";${TICK}:${WHOLE}.${FRACTION},0,1")
    else()
      string(APPEND WAYPOINTS ";${TICK}:0,${WHOLE}.${FRACTION},1")
    endif()
  endforeach()
  configure_file(configuration.argos.in ${AXIS}.argos)
  add_test(NAME jolt_drone_tracking_${AXIS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc ${AXIS}.argos)
  set_tests_properties(jolt_drone_tracking_${AXIS}
    PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
endforeach()
