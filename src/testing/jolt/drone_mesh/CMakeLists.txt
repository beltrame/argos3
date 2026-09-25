# The drone against static mesh geometry. The corridor is the mesh suite's
# generated asset; the SubT case runs only where its collision mesh exists.
add_library(drone_mesh_jolt_controller MODULE
  controller.h
  controller.cpp)
target_link_libraries(drone_mesh_jolt_controller
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_drone)
add_library(drone_mesh_jolt_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_link_libraries(drone_mesh_jolt_loop_functions
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_entities
  argos3plugin_${ARGOS_BUILD_FOR}_drone)

if(TARGET mesh_jolt_assets)
  set(DRONE_MESH_CORRIDOR ${CMAKE_BINARY_DIR}/testing/jolt/mesh/corridor.glb)
  add_dependencies(drone_mesh_jolt_loop_functions mesh_jolt_assets)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/corridor.argos.in
    ${CMAKE_CURRENT_BINARY_DIR}/corridor.argos)
  add_test(
    NAME jolt_drone_mesh_corridor
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc corridor.argos)
  set_tests_properties(jolt_drone_mesh_corridor
    PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
endif()

set(ARGOS_SUBT_COLLISION_GLB
  "${CMAKE_SOURCE_DIR}/plugins/simulator/photorealism/environments/finals_prize_round_world_01/finals_prize_round_world_01.collision.glb"
  CACHE FILEPATH "SubT Finals collision mesh for jolt_drone_mesh_subt")
if(EXISTS "${ARGOS_SUBT_COLLISION_GLB}")
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/subt.argos.in
    ${CMAKE_CURRENT_BINARY_DIR}/subt.argos)
  add_test(
    NAME jolt_drone_mesh_subt
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc subt.argos)
  set_tests_properties(jolt_drone_mesh_subt
    PROPERTIES LABELS "subt" ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
endif()
