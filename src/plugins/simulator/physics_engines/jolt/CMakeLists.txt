#
# Jolt physics engine plugin
#
# Jolt (https://github.com/jrouwe/JoltPhysics) is downloaded at a
# pinned tag and statically linked into the plugin.
#
include(FetchContent)
FetchContent_Declare(joltphysics
  URL https://github.com/jrouwe/JoltPhysics/archive/refs/tags/v5.2.0.tar.gz
  URL_HASH ""
  SOURCE_SUBDIR Build)
# Bitwise determinism across platforms and runs
set(CROSS_PLATFORM_DETERMINISTIC ON CACHE BOOL "" FORCE)
# No need for the extra targets, debug symbols, or LTO
set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)
set(GENERATE_DEBUG_SYMBOLS OFF CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(joltphysics)
# Jolt is linked into this shared plugin
set_property(TARGET Jolt PROPERTY POSITION_INDEPENDENT_CODE ON)

#
# Headers
#
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_JOLT
  jolt_common.h
  jolt_engine.h
  jolt_model.h
  jolt_single_body_object_model.h
  jolt_shape_manager.h
  jolt_plugin.h
  jolt_floor_plugin.h
  jolt_gravity_plugin.h
  jolt_box_model.h
  jolt_cylinder_model.h
  jolt_mesh_asset.h
  jolt_mesh_model.h
  mesh_entity.h)

#
# Source files
#
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_JOLT
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_JOLT}
  jolt_engine.cpp
  jolt_model.cpp
  jolt_single_body_object_model.cpp
  jolt_shape_manager.cpp
  jolt_floor_plugin.cpp
  jolt_gravity_plugin.cpp
  jolt_box_model.cpp
  jolt_cylinder_model.cpp
  jolt_mesh_asset.cpp
  jolt_mesh_model.cpp
  mesh_entity.cpp)

#
# Create the jolt engine plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_jolt SHARED
  ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_JOLT})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_jolt
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_entities
  Jolt)

install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_JOLT}
  DESTINATION include/argos3/plugins/simulator/physics_engines/jolt)
install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_jolt
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)

#
# Add plugin to ARGOS_PLUGIN_PATH
#
set(ARGOS_PLUGIN_PATH "${ARGOS_PLUGIN_PATH}:${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "ARGoS plugin path")
