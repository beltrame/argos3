# The glTF assets these tests run against are generated at build time, so
# that no binary file has to be kept in the repository.
find_package(Python3 COMPONENTS Interpreter)
if(NOT Python3_Interpreter_FOUND)
  message(STATUS "Python 3 not found: skipping the Jolt mesh tests")
  return()
endif(NOT Python3_Interpreter_FOUND)
set(WALL_ASSETS ${CMAKE_CURRENT_BINARY_DIR}/wall_plain.glb
                ${CMAKE_CURRENT_BINARY_DIR}/wall_lip.glb
                ${CMAKE_CURRENT_BINARY_DIR}/ramp16.glb
                ${CMAKE_CURRENT_BINARY_DIR}/ramp18.glb
                ${CMAKE_CURRENT_BINARY_DIR}/step10.glb
                ${CMAKE_CURRENT_BINARY_DIR}/step20.glb
                ${CMAKE_CURRENT_BINARY_DIR}/step30.glb)
add_custom_command(
  OUTPUT ${WALL_ASSETS}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/make_wall_meshes.py
          ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS make_wall_meshes.py make_meshes.py)
add_custom_target(mesh_jolt_wall_assets ALL DEPENDS ${WALL_ASSETS})
set(MESH_ASSETS
  ${CMAKE_CURRENT_BINARY_DIR}/corridor.glb
  ${CMAKE_CURRENT_BINARY_DIR}/flipped_wall.glb
  ${CMAKE_CURRENT_BINARY_DIR}/tiled_floor.glb
  ${CMAKE_CURRENT_BINARY_DIR}/terrain.glb)
add_custom_command(
  OUTPUT ${MESH_ASSETS}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/make_meshes.py
          ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/make_meshes.py
  COMMENT "Generating the glTF assets of the Jolt mesh tests")
add_custom_target(mesh_jolt_assets ALL DEPENDS ${MESH_ASSETS})
# compile test controller
add_library(mesh_jolt_controller MODULE
  controller.h
  controller.cpp)
target_link_libraries(mesh_jolt_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)
# compile test loop functions
add_library(mesh_jolt_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_link_libraries(mesh_jolt_loop_functions
    argos3core_${ARGOS_BUILD_FOR})
foreach(ATTITUDE_PLUGIN bunker scoutmini spot)
  if(TARGET argos3plugin_${ARGOS_BUILD_FOR}_${ATTITUDE_PLUGIN})
    add_dependencies(mesh_jolt_loop_functions
      argos3plugin_${ARGOS_BUILD_FOR}_${ATTITUDE_PLUGIN})
  endif()
endforeach()
set(ATTITUDE_ROBOT bunker)
set(ATTITUDE_ID bunker_attitude)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/attitude.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/bunker_attitude.argos)
set(ATTITUDE_ROBOT scout_mini)
set(ATTITUDE_ID scout_mini_attitude)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/attitude.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/scout_mini_attitude.argos)
set(ATTITUDE_ROBOT spot)
set(ATTITUDE_ID spot_attitude)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/attitude.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/spot_attitude.argos)
add_library(mesh_jolt_support_contacts MODULE support_contacts.cpp)
target_link_libraries(mesh_jolt_support_contacts
  argos3plugin_${ARGOS_BUILD_FOR}_jolt)
configure_file(support_contacts.argos.in support_contacts.argos)
add_test(NAME jolt_mesh_spot_support_contacts
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc support_contacts.argos)
set_tests_properties(jolt_mesh_spot_support_contacts PROPERTIES
  ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
# Wheel/track climbing remains physical; only Spot's side contacts are passive.
foreach(WALL_ROBOT bunker scout_mini spot)
  set(WALL_FRICTION 1.0)
  set(WALL_CHECKS "")
  set(WALL_TILT 180)
  if(WALL_ROBOT STREQUAL "spot")
    set(WALL_TILT 10)
  endif()
  foreach(WALL_SPEED 10 60)
    foreach(WALL_YAW 0 45)
      foreach(WALL_KIND plain lip)
        set(WALL_CASE ${WALL_ROBOT}_wall_push_${WALL_SPEED}_${WALL_YAW}_${WALL_KIND})
        configure_file(wall_push.argos.in ${WALL_CASE}.argos)
        add_test(NAME jolt_mesh_${WALL_CASE}
          WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
          COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc ${WALL_CASE}.argos)
        set_tests_properties(jolt_mesh_${WALL_CASE} PROPERTIES
          ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
      endforeach()
    endforeach()
  endforeach()
  # A surface-velocity command is not a chassis-velocity overwrite: with zero
  # friction a supported robot cannot accelerate, even at full command.
  set(WALL_FRICTION 0.0)
  set(WALL_YAW 0)
  set(WALL_KIND plain)
  set(WALL_SPEED 60)
  set(WALL_CHECKS "expect_x=\"0\" position_tolerance=\"0.001\" maximum_speed=\"0.001\"")
  configure_file(wall_push.argos.in ${WALL_ROBOT}_no_friction.argos)
  add_test(NAME jolt_mesh_${WALL_ROBOT}_no_friction
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc ${WALL_ROBOT}_no_friction.argos)
  set_tests_properties(jolt_mesh_${WALL_ROBOT}_no_friction PROPERTIES
    ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
endforeach()
include(terrain_tests.cmake)
# configure experiments
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/rays.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/rays.argos)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cache.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/cache.argos)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ramp.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/ramp.argos)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/traction.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/traction.argos)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/slide.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/slide.argos)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bench.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/bench.argos)
# two identical collision experiments writing different pose files
set(POSES_FILE poses_1.txt)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/collision.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/collision_1.argos)
set(POSES_FILE poses_2.txt)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/collision.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/collision_2.argos)
# the same wrongly wound wall with double-sided loading on and off
set(DOUBLE_SIDED true)
set(EXPECT_X 26.914963)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/winding.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/winding_double_sided.argos)
set(DOUBLE_SIDED false)
set(EXPECT_X 29.914963)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/winding.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/winding_single_sided.argos)
# ray distances against a mesh of known geometry
add_test(
   NAME jolt_mesh_rays
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc rays.argos)
# one asset, six entities: shape cache, scale, y_up, double_sided
add_test(
   NAME jolt_mesh_cache
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc cache.argos)
# a robot driven into a mesh wall stops at the wall
add_test(
   NAME jolt_mesh_collision
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc collision_1.argos)
# a robot sliding over a tessellated mesh surface is not deflected by the
# internal edges of the triangulation
add_test(
   NAME jolt_mesh_slide
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc slide.argos)
# a robot released above a sloped mesh surface rests on it
add_test(
   NAME jolt_mesh_ramp
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc ramp.argos)
# real Bunker, Scout Mini and Spot bodies drive uphill over the ramp while
# their body attitude and differential yaw response are checked
add_test(
   NAME jolt_mesh_bunker_attitude
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc bunker_attitude.argos)
add_test(
   NAME jolt_mesh_scout_mini_attitude
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc scout_mini_attitude.argos)
add_test(
   NAME jolt_mesh_spot_attitude
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc spot_attitude.argos)
# Without ground contact, wheel commands cannot supply chassis propulsion.
add_test(
   NAME jolt_mesh_traction
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc traction.argos)
# a wrongly wound wall stops the robot with double-sided loading on and lets
# it through with double-sided loading off, while rays hit it either way
add_test(
   NAME jolt_mesh_winding
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "${CMAKE_BINARY_DIR}/core/argos3 -zc winding_double_sided.argos && ${CMAKE_BINARY_DIR}/core/argos3 -zc winding_single_sided.argos")
# bitwise determinism: two runs of the collision scenario must produce
# identical final poses
add_test(
   NAME jolt_mesh_determinism
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND bash -c "${CMAKE_BINARY_DIR}/core/argos3 -zc collision_1.argos && ${CMAKE_BINARY_DIR}/core/argos3 -zc collision_2.argos && cmp poses_1.txt poses_2.txt")
# ray accuracy and throughput against a large mesh
add_test(
   NAME jolt_mesh_bench
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND ${CMAKE_BINARY_DIR}/core/argos3 -zc bench.argos)
set_tests_properties(jolt_mesh_rays jolt_mesh_cache jolt_mesh_collision
  jolt_mesh_slide jolt_mesh_ramp jolt_mesh_bunker_attitude
  jolt_mesh_scout_mini_attitude jolt_mesh_spot_attitude jolt_mesh_traction
  jolt_mesh_winding jolt_mesh_determinism jolt_mesh_bench
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")
