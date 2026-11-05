#
# The media plugin resolves the entities' grid updaters at load time, from
# the entities plugin. On Linux, keep the entities plugin linked even though
# these executables use none of its symbols, as ARGoS does for its shared
# libraries (cmake/ARGoSBuildFlags.cmake); macOS already looks them up
# dynamically.
#
set(MESH_TEST_LIBRARIES
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_entities
  argos3plugin_${ARGOS_BUILD_FOR}_media)
if(NOT APPLE)
  set(MESH_TEST_LINK_OPTIONS "LINKER:--no-as-needed")
endif()

#
# The routing on its own
#
add_executable(test_mesh_routes test_mesh_routes.cpp)
target_link_libraries(test_mesh_routes ${MESH_TEST_LIBRARIES})
target_link_options(test_mesh_routes PRIVATE ${MESH_TEST_LINK_OPTIONS})
add_test(NAME mesh_medium_routes COMMAND test_mesh_routes)

#
# The medium inside an experiment, with attributes at and off their defaults.
# The executable links the media plugin itself; the argos3 runs below load
# it by name from ARGOS_PLUGIN_PATH.
#
add_executable(test_mesh_medium test_mesh_medium.cpp)
target_link_libraries(test_mesh_medium ${MESH_TEST_LIBRARIES})
target_link_options(test_mesh_medium PRIVATE ${MESH_TEST_LINK_OPTIONS})
set(MESH_EXPERIMENT_CASES
  "limited|robots=\"robot_0,robot_1\" range_m=\"45\" base_position=\"-40,0,0\""
  "unlimited|robots=\"robot_0,robot_1\" range_m=\"30\" unlimited=\"true\" cap_bandwidth=\"false\" base_id=\"gateway\" base_position=\"-40,0,0\"")
foreach(_case ${MESH_EXPERIMENT_CASES})
  string(REPLACE "|" ";" _fields "${_case}")
  list(GET _fields 0 _mode)
  list(GET _fields 1 MESH_ATTRIBUTES)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
    ${CMAKE_CURRENT_BINARY_DIR}/${_mode}.argos)
  add_test(NAME mesh_medium_experiment_${_mode}
    COMMAND test_mesh_medium ${CMAKE_CURRENT_BINARY_DIR}/${_mode}.argos ${_mode})
  set_tests_properties(mesh_medium_experiment_${_mode}
    PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
endforeach()

#
# Configurations the medium must refuse. Each test passes only when argos3
# reports the expected reason as a fatal error.
#
set(MESH_INVALID_CASES
  "no_robots|robots=\",\" base_position=\"0,0,0\"|no robots listed"
  "zero_range|robots=\"robot_0\" range_m=\"0\" base_position=\"0,0,0\"|range_m must be finite and positive"
  "uncapped_limited|robots=\"robot_0\" cap_bandwidth=\"false\" base_position=\"0,0,0\"|cap_bandwidth=\"false\" needs unlimited=\"true\""
  "reserved_id|robots=\"robot_0,base_station\" base_position=\"0,0,0\"|robot id \"base_station\" is reserved for the base station"
  "unknown_robot|robots=\"robot_0,ghost\" base_position=\"0,0,0\"|Error locating mesh robot \"ghost\""
  "no_base_position|robots=\"robot_0\"|Error parsing attribute \"base_position\"")
foreach(_case ${MESH_INVALID_CASES})
  string(REPLACE "|" ";" _fields "${_case}")
  list(GET _fields 0 _name)
  list(GET _fields 1 MESH_ATTRIBUTES)
  list(GET _fields 2 _reason)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/configuration.argos.in
    ${CMAKE_CURRENT_BINARY_DIR}/invalid_${_name}.argos)
  add_test(NAME mesh_medium_rejects_${_name}
    COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc ${CMAKE_CURRENT_BINARY_DIR}/invalid_${_name}.argos)
  set_tests_properties(mesh_medium_rejects_${_name} PROPERTIES
    ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}"
    PASS_REGULAR_EXPRESSION "\\[FATAL\\] ${_reason}")
endforeach()
