#
# Bunker headers
#
# argos3/plugins/robots/bunker/simulator
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKER_SIMULATOR
    simulator/dynamics2d_bunker_model.h
    simulator/bunker_entity.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Bunker sources
#
# No control_interface: the platform is driven through the generic
# differential_steering actuator, and no qtopengl model, because SwarmDeck
# renders through the Filament photorealism medium and a wrong-looking hull in
# the legacy viewer is worse than no entry at all.
#
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_BUNKER
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKER_SIMULATOR}
    simulator/dynamics2d_bunker_model.cpp
    simulator/bunker_entity.cpp)

  # Compile Jolt model when Jolt engine is built
  if(TARGET argos3plugin_${ARGOS_BUILD_FOR}_jolt)
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKER_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKER_SIMULATOR}
      simulator/jolt_bunker_model.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_BUNKER
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_BUNKER}
      simulator/jolt_bunker_model.h
      simulator/jolt_bunker_model.cpp)
  endif()
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create bunker plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_bunker SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_BUNKER})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_bunker
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)

if(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_bunker
    argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d
    argos3plugin_${ARGOS_BUILD_FOR}_entities)
  if(TARGET argos3plugin_${ARGOS_BUILD_FOR}_jolt)
    target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_bunker
      argos3plugin_${ARGOS_BUILD_FOR}_jolt)
  endif()
endif(ARGOS_BUILD_FOR_SIMULATOR)

set(ARGOS_PLUGIN_PATH "${ARGOS_PLUGIN_PATH}:${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "ARGoS plugin path")

#
# Install plugin
#
install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_bunker
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_BUNKER_SIMULATOR}
    DESTINATION include/argos3/plugins/robots/bunker/simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)
