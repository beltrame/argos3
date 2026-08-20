#
# Headers
#
# argos3/plugins/simulator/external_estimator/
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_EXTERNAL_ESTIMATOR
  external_estimator_medium.h
  odometry_external_sensor.h)

#
# Source files
#
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_EXTERNAL_ESTIMATOR
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_EXTERNAL_ESTIMATOR}
  external_estimator_medium.cpp
  odometry_external_sensor.cpp)

#
# Create the external estimator plugin.
#
# The medium and the sensor ship together: the sensor resolves the
# medium by name through the simulator, so both must be loaded for
# either to be useful.
#
# Only the control interfaces of the camera and lidar sensors are used
# (they live in the genericrobot plugin), so this plugin builds with or
# without Filament.
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_externalestimator SHARED
  ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_EXTERNAL_ESTIMATOR})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_externalestimator
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_entities
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  Threads::Threads)

#
# Add plugin to ARGOS_PLUGIN_PATH
#
set(ARGOS_PLUGIN_PATH "${ARGOS_PLUGIN_PATH}:${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "ARGoS plugin path")

#
# Installation
#
install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_EXTERNAL_ESTIMATOR}
  DESTINATION include/argos3/plugins/simulator/external_estimator/)
install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_externalestimator
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
