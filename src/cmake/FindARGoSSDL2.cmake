#
# Find SDL2 for the Filament visualization.
#
# Works without the distro dev package: headers can be extracted
# locally (apt-get download libsdl2-dev; dpkg -x ... root/) next to
# the Filament SDK, and the runtime library is accepted directly.
#
# Sets:
#   ARGOS_SDL2_FOUND
#   ARGOS_SDL2_INCLUDE_DIRS
#   ARGOS_SDL2_LIBRARIES
#

set(_ARGOS_SDL2_HINTS)
if(DEFINED SDL2_DIR)
  list(APPEND _ARGOS_SDL2_HINTS ${SDL2_DIR}/include)
endif()
if(DEFINED ENV{SDL2_DIR})
  list(APPEND _ARGOS_SDL2_HINTS $ENV{SDL2_DIR}/include)
endif()
if(DEFINED FILAMENT_DIR)
  list(APPEND _ARGOS_SDL2_HINTS ${FILAMENT_DIR}/../sdl2/root/usr/include)
endif()

find_path(ARGOS_SDL2_INCLUDE_DIR
  NAMES SDL2/SDL.h
  HINTS ${_ARGOS_SDL2_HINTS})

find_library(ARGOS_SDL2_LIBRARY
  NAMES SDL2 SDL2-2.0 libSDL2-2.0.so.0
  HINTS ${SDL2_DIR}/lib $ENV{SDL2_DIR}/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ARGoSSDL2
  REQUIRED_VARS ARGOS_SDL2_INCLUDE_DIR ARGOS_SDL2_LIBRARY)

if(ARGOSSDL2_FOUND)
  set(ARGOS_SDL2_FOUND TRUE)
  set(ARGOS_SDL2_INCLUDE_DIRS ${ARGOS_SDL2_INCLUDE_DIR})
  set(ARGOS_SDL2_LIBRARIES ${ARGOS_SDL2_LIBRARY})
  # Debian splits the SDL configuration header into an
  # architecture-specific directory
  if(EXISTS ${ARGOS_SDL2_INCLUDE_DIR}/${CMAKE_LIBRARY_ARCHITECTURE}/SDL2/_real_SDL_config.h)
    list(APPEND ARGOS_SDL2_INCLUDE_DIRS
      ${ARGOS_SDL2_INCLUDE_DIR}/${CMAKE_LIBRARY_ARCHITECTURE})
  endif()
endif()

mark_as_advanced(ARGOS_SDL2_INCLUDE_DIR ARGOS_SDL2_LIBRARY)
