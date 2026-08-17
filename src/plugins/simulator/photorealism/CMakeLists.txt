#
# Headers
#
# argos3/plugins/simulator/photorealism/
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHOTOREALISM
  photorealism_medium.h)
# argos3/plugins/simulator/photorealism/render_core/
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHOTOREALISM_RENDERCORE
  render_core/pr_render_engine.h
  render_core/pr_mesh_builder.h
  render_core/pr_scene_sync.h
  render_core/pr_id_scene.h
  render_core/pr_camera_pool.h
  render_core/pr_debug_draw.h
  render_core/pr_randomizer.h
  render_core/pr_asset_registry.h)
# argos3/plugins/simulator/photorealism/sensors/
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHOTOREALISM_SENSORS
  sensors/photorealistic_camera_default_sensor.h
  sensors/photorealistic_lidar_default_sensor.h)

#
# Compile the Filament materials with matc and embed them as C++ arrays
#
set(ARGOS3_PHOTOREALISM_MATERIALS pr_lit pr_aux pr_lit_tex pr_blit pr_overlay)
set(ARGOS3_PHOTOREALISM_MATERIAL_SOURCES "")
foreach(_mat ${ARGOS3_PHOTOREALISM_MATERIALS})
  string(TOUPPER ${_mat} _MAT_SYMBOL)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_mat}.filamat
    COMMAND ${FILAMENT_MATC} -a all -o ${CMAKE_CURRENT_BINARY_DIR}/${_mat}.filamat
            ${CMAKE_CURRENT_SOURCE_DIR}/materials/${_mat}.mat
    MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/materials/${_mat}.mat
    COMMENT "Compiling Filament material ${_mat}.mat")
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_mat}_filamat.cpp
    COMMAND ${CMAKE_COMMAND}
            -DINPUT=${CMAKE_CURRENT_BINARY_DIR}/${_mat}.filamat
            -DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/${_mat}_filamat.cpp
            -DSYMBOL=${_MAT_SYMBOL}_FILAMAT
            -P ${CMAKE_SOURCE_DIR}/cmake/ARGoSHexifyFile.cmake
    MAIN_DEPENDENCY ${CMAKE_CURRENT_BINARY_DIR}/${_mat}.filamat
    COMMENT "Embedding Filament material ${_mat}.filamat")
  list(APPEND ARGOS3_PHOTOREALISM_MATERIAL_SOURCES
    ${CMAKE_CURRENT_BINARY_DIR}/${_mat}_filamat.cpp)
endforeach()

#
# Source files
#
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHOTOREALISM
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHOTOREALISM}
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHOTOREALISM_RENDERCORE}
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHOTOREALISM_SENSORS}
  ${ARGOS3_PHOTOREALISM_MATERIAL_SOURCES}
  photorealism_medium.cpp
  render_core/pr_render_engine.cpp
  render_core/pr_mesh_builder.cpp
  render_core/pr_scene_sync.cpp
  render_core/pr_id_scene.cpp
  render_core/pr_camera_pool.cpp
  render_core/pr_debug_draw.cpp
  render_core/pr_randomizer.cpp
  render_core/pr_asset_registry.cpp
  sensors/photorealistic_camera_default_sensor.cpp
  sensors/photorealistic_lidar_default_sensor.cpp)

#
# Symbol firewall: pre-link the Filament archives together with the
# static libc++/libc++abi runtime into a single relocatable object,
# resolving all their internal references, then localize every symbol
# except the Filament public API. Without this, the libc++abi runtime
# symbols (__dynamic_cast, __cxa_*, operator new, ...) would satisfy
# references from the GCC/libstdc++-compiled plugin code and crash at
# the first dynamic_cast on an ARGoS type.
#
set(ARGOS3_FILAMENT_BUNDLE ${CMAKE_CURRENT_BINARY_DIR}/filament_bundle)
add_custom_command(
  OUTPUT ${ARGOS3_FILAMENT_BUNDLE}.a
  COMMAND ${CMAKE_LINKER} -r -o ${ARGOS3_FILAMENT_BUNDLE}.o
          --whole-archive ${FILAMENT_CORE_LIBRARIES}
          ${FILAMENT_gltfio_core_LIBRARY}
          ${FILAMENT_uberarchive_LIBRARY}
          ${FILAMENT_ktxreader_LIBRARY}
          ${FILAMENT_image_LIBRARY}
          --no-whole-archive ${FILAMENT_GLTFIO_LIBRARIES}
          ${FILAMENT_LIBCXX_LIBRARIES}
  COMMAND ${CMAKE_OBJCOPY} -w
          --keep-global-symbol=_ZN8filament*
          --keep-global-symbol=_ZNK8filament*
          --keep-global-symbol=_ZTVN8filament*
          --keep-global-symbol=_ZTIN8filament*
          --keep-global-symbol=_ZTSN8filament*
          --keep-global-symbol=_ZN5utils*
          --keep-global-symbol=_ZNK5utils*
          --keep-global-symbol=_ZTVN5utils*
          --keep-global-symbol=_ZTIN5utils*
          --keep-global-symbol=_ZTSN5utils*
          --keep-global-symbol=_ZN6gltfio*
          --keep-global-symbol=_ZNK6gltfio*
          --keep-global-symbol=_ZTVN6gltfio*
          --keep-global-symbol=_ZTIN6gltfio*
          --keep-global-symbol=_ZTSN6gltfio*
          --keep-global-symbol=_ZN9ktxreader*
          --keep-global-symbol=_ZNK9ktxreader*
          --keep-global-symbol=_ZN5image*
          --keep-global-symbol=_ZNK5image*
          --keep-global-symbol=UBERARCHIVE_PACKAGE
          ${ARGOS3_FILAMENT_BUNDLE}.o
  COMMAND ${CMAKE_AR} rcs ${ARGOS3_FILAMENT_BUNDLE}.a ${ARGOS3_FILAMENT_BUNDLE}.o
  DEPENDS ${FILAMENT_CORE_LIBRARIES} ${FILAMENT_GLTFIO_LIBRARIES}
  COMMENT "Creating Filament symbol-firewalled bundle")
add_custom_target(argos3_filament_bundle DEPENDS ${ARGOS3_FILAMENT_BUNDLE}.a)

#
# Create the photorealism plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_photorealism SHARED
  ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHOTOREALISM})
add_dependencies(argos3plugin_${ARGOS_BUILD_FOR}_photorealism
  argos3_filament_bundle)
target_include_directories(argos3plugin_${ARGOS_BUILD_FOR}_photorealism
  PRIVATE ${FILAMENT_INCLUDE_DIRS})
# Default search directory for the installed visual assets
target_compile_definitions(argos3plugin_${ARGOS_BUILD_FOR}_photorealism
  PRIVATE ARGOS_PR_ASSET_DIR="${CMAKE_INSTALL_PREFIX}/share/argos3/photorealism")
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_photorealism
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_entities
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  ${ARGOS3_FILAMENT_BUNDLE}.a
  Threads::Threads
  ${CMAKE_DL_LIBS})
if(UNIX AND NOT APPLE)
  # Keep the Filament API symbols out of the plugin's dynamic symbol
  # table to avoid clashes with other plugins
  target_link_options(argos3plugin_${ARGOS_BUILD_FOR}_photorealism
    PRIVATE "LINKER:--exclude-libs,ALL")
endif()

#
# Add plugin to ARGOS_PLUGIN_PATH
#
set(ARGOS_PLUGIN_PATH "${ARGOS_PLUGIN_PATH}:${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "ARGoS plugin path")

#
# Installation
#
install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHOTOREALISM}
  DESTINATION include/argos3/plugins/simulator/photorealism/)
install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHOTOREALISM_RENDERCORE}
  DESTINATION include/argos3/plugins/simulator/photorealism/render_core/)
install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHOTOREALISM_SENSORS}
  DESTINATION include/argos3/plugins/simulator/photorealism/sensors/)
install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_photorealism
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
install(DIRECTORY assets/
  DESTINATION share/argos3/photorealism)
