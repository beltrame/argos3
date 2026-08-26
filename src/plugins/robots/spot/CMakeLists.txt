#
# Spot headers
#
# argos3/plugins/robots/spot/simulator
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_SPOT_SIMULATOR
    simulator/dynamics2d_spot_model.h
    simulator/spot_entity.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Spot sources
#
# No control_interface: the platform is driven through the generic
# differential_steering actuator, and no qtopengl model, because SwarmDeck
# renders through the Filament photorealism medium and a wrong-looking hull in
# the legacy viewer is worse than no entry at all.
#
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_SPOT
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_SPOT_SIMULATOR}
    simulator/dynamics2d_spot_model.cpp
    simulator/spot_entity.cpp)

  # Compile Jolt model when Jolt engine is built
  if(TARGET argos3plugin_${ARGOS_BUILD_FOR}_jolt)
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_SPOT_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_SPOT_SIMULATOR}
      simulator/jolt_spot_model.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_SPOT
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_SPOT}
      simulator/jolt_spot_model.h
      simulator/jolt_spot_model.cpp)
  endif()
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create spot plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_spot SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_SPOT})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_spot
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)

if(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_spot
    argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d
    argos3plugin_${ARGOS_BUILD_FOR}_entities)
  if(TARGET argos3plugin_${ARGOS_BUILD_FOR}_jolt)
    target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_spot
      argos3plugin_${ARGOS_BUILD_FOR}_jolt)
  endif()
endif(ARGOS_BUILD_FOR_SIMULATOR)

set(ARGOS_PLUGIN_PATH "${ARGOS_PLUGIN_PATH}:${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "ARGoS plugin path")

#
# Install plugin
#
install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_spot
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_SPOT_SIMULATOR}
    DESTINATION include/argos3/plugins/robots/spot/simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)
