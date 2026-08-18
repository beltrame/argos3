#
# Bunker Mini headers
#
# argos3/plugins/robots/bunker-mini/control_interface
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKERMINI_CONTROLINTERFACE
  control_interface/ci_bunker_mini_track_actuator.h
  control_interface/ci_bunker_mini_track_sensor.h)

# argos3/plugins/robots/bunker-mini/simulator
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKERMINI_SIMULATOR
    simulator/dynamics2d_bunker_mini_model.h
    simulator/bunker_mini_entity.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Bunker Mini sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_BUNKERMINI
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKERMINI_CONTROLINTERFACE}
  control_interface/ci_bunker_mini_track_actuator.cpp
  control_interface/ci_bunker_mini_track_sensor.cpp)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_BUNKERMINI
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_BUNKERMINI}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKERMINI_SIMULATOR}
    simulator/dynamics2d_bunker_mini_model.cpp
    simulator/bunker_mini_entity.cpp)
  
  # Compile Jolt model when Jolt engine is built
  if(TARGET argos3plugin_${ARGOS_BUILD_FOR}_jolt)
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKERMINI_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKERMINI_SIMULATOR}
      simulator/jolt_bunker_mini_model.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_BUNKERMINI
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_BUNKERMINI}
      simulator/jolt_bunker_mini_model.h
      simulator/jolt_bunker_mini_model.cpp)
  endif()

  # Compile QtOpenGL visualization when QtOpenGL is available
  if(ARGOS_QTOPENGL_FOUND)
    include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKERMINI_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKERMINI_SIMULATOR}
      simulator/qtopengl_bunker_mini.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_BUNKERMINI
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_BUNKERMINI}
      simulator/qtopengl_bunker_mini.h
      simulator/qtopengl_bunker_mini.cpp)
  endif(ARGOS_QTOPENGL_FOUND)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create bunker-mini plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_bunkermini SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_BUNKERMINI})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_bunkermini
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)

if(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_bunkermini
    argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d
    argos3plugin_${ARGOS_BUILD_FOR}_entities)
  if(TARGET argos3plugin_${ARGOS_BUILD_FOR}_jolt)
    target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_bunkermini
      argos3plugin_${ARGOS_BUILD_FOR}_jolt)
  endif()
  if(ARGOS_QTOPENGL_FOUND)
    target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_bunkermini
      argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
  endif(ARGOS_QTOPENGL_FOUND)
endif(ARGOS_BUILD_FOR_SIMULATOR)

set(ARGOS_PLUGIN_PATH "${ARGOS_PLUGIN_PATH}:${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "ARGoS plugin path")

#
# Install plugin
#
install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_bunkermini
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKERMINI_CONTROLINTERFACE}
  DESTINATION include/argos3/plugins/robots/bunker-mini/control_interface)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKERMINI_SIMULATOR}
    DESTINATION include/argos3/plugins/robots/bunker-mini/simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)
