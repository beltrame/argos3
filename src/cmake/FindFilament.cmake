# Find the Google Filament rendering engine SDK.
#
# Filament is distributed as a set of static libraries plus the matc
# material compiler:
#   https://github.com/google/filament/releases
#
# This module looks for the SDK root in:
#   1. The CMake variable FILAMENT_DIR
#   2. The environment variable FILAMENT_DIR
#
# The official Linux prebuilts are compiled with clang against libc++.
# They can be linked from a GCC/libstdc++ build as long as the static
# libc++/libc++abi runtime archives are linked in as well (the two
# standard libraries use different mangled namespaces and coexist in
# one binary). The runtime archives are looked for in:
#   1. The CMake variable FILAMENT_LIBCXX_DIR
#   2. <FILAMENT_DIR>/../libcxx/root/usr/lib/llvm-*/lib
#   3. The system library directories
# On Linux, if they cannot be found, Filament is reported as not found.
#
# This module sets:
#   FILAMENT_FOUND
#   FILAMENT_INCLUDE_DIRS
#   FILAMENT_LIBRARIES     (Filament static libs + libc++ runtime, in link order)
#   FILAMENT_MATC          (path to the matc material compiler)

if(NOT FILAMENT_DIR)
  set(FILAMENT_DIR "$ENV{FILAMENT_DIR}")
endif(NOT FILAMENT_DIR)

set(FILAMENT_FOUND OFF)

if(FILAMENT_DIR)
  find_path(FILAMENT_INCLUDE_DIR
    NAMES filament/Engine.h
    PATHS ${FILAMENT_DIR}/include
    NO_DEFAULT_PATH)
  find_program(FILAMENT_MATC
    NAMES matc
    PATHS ${FILAMENT_DIR}/bin
    NO_DEFAULT_PATH)

  # The order matters for static linking; --start/end-group is added by
  # the caller through ARGOS_START_LIB_GROUP/ARGOS_END_LIB_GROUP.
  set(_FILAMENT_LIB_NAMES
    filament backend bluevk bluegl filaflat filabridge ibl utils
    geometry smol-v shaders perfetto abseil zstd)
  # Extra libraries needed by gltfio (used from M3 on); listed after the
  # core set so they are available as soon as they are needed.
  set(_FILAMENT_GLTFIO_LIB_NAMES
    gltfio_core dracodec meshoptimizer stb uberzlib uberarchive
    ktxreader image basis_transcoder)

  set(FILAMENT_CORE_LIBRARIES "")
  set(FILAMENT_GLTFIO_LIBRARIES "")
  set(_FILAMENT_ALL_CORE_FOUND ON)
  foreach(_lib ${_FILAMENT_LIB_NAMES} ${_FILAMENT_GLTFIO_LIB_NAMES})
    find_library(FILAMENT_${_lib}_LIBRARY
      NAMES ${_lib}
      PATHS ${FILAMENT_DIR}/lib/x86_64 ${FILAMENT_DIR}/lib/arm64 ${FILAMENT_DIR}/lib
      NO_DEFAULT_PATH)
    mark_as_advanced(FILAMENT_${_lib}_LIBRARY)
    if(FILAMENT_${_lib}_LIBRARY)
      if(${_lib} IN_LIST _FILAMENT_LIB_NAMES)
        list(APPEND FILAMENT_CORE_LIBRARIES ${FILAMENT_${_lib}_LIBRARY})
      else()
        list(APPEND FILAMENT_GLTFIO_LIBRARIES ${FILAMENT_${_lib}_LIBRARY})
      endif()
    else()
      set(_FILAMENT_ALL_CORE_FOUND OFF)
      message(STATUS "Filament: missing library '${_lib}' under ${FILAMENT_DIR}/lib")
    endif()
  endforeach()

  # libc++ runtime archives (Linux only; on macOS libc++ is the system default)
  set(FILAMENT_LIBCXX_LIBRARIES "")
  set(_FILAMENT_LIBCXX_OK ON)
  if(UNIX AND NOT APPLE)
    set(_FILAMENT_LIBCXX_OK OFF)
    file(GLOB _libcxx_glob "${FILAMENT_DIR}/../libcxx/root/usr/lib/llvm-*/lib")
    find_library(FILAMENT_LIBCXX_LIBRARY
      NAMES libc++.a c++
      PATHS ${FILAMENT_LIBCXX_DIR} ${_libcxx_glob}
      NO_DEFAULT_PATH)
    find_library(FILAMENT_LIBCXXABI_LIBRARY
      NAMES libc++abi.a c++abi
      PATHS ${FILAMENT_LIBCXX_DIR} ${_libcxx_glob}
      NO_DEFAULT_PATH)
    if(NOT FILAMENT_LIBCXX_LIBRARY)
      find_library(FILAMENT_LIBCXX_LIBRARY NAMES libc++.a c++)
      find_library(FILAMENT_LIBCXXABI_LIBRARY NAMES libc++abi.a c++abi)
    endif()
    mark_as_advanced(FILAMENT_LIBCXX_LIBRARY FILAMENT_LIBCXXABI_LIBRARY)
    if(FILAMENT_LIBCXX_LIBRARY AND FILAMENT_LIBCXXABI_LIBRARY)
      set(FILAMENT_LIBCXX_LIBRARIES ${FILAMENT_LIBCXX_LIBRARY} ${FILAMENT_LIBCXXABI_LIBRARY})
      set(_FILAMENT_LIBCXX_OK ON)
    else()
      message(STATUS "Filament: libc++/libc++abi runtime not found (set FILAMENT_LIBCXX_DIR); Filament plugins will be skipped")
    endif()
  endif()

  if(FILAMENT_INCLUDE_DIR AND FILAMENT_MATC AND _FILAMENT_ALL_CORE_FOUND AND _FILAMENT_LIBCXX_OK)
    set(FILAMENT_FOUND ON)
    set(FILAMENT_INCLUDE_DIRS ${FILAMENT_INCLUDE_DIR})
    find_package(Threads REQUIRED)
    set(FILAMENT_LIBRARIES
      ${FILAMENT_CORE_LIBRARIES}
      ${FILAMENT_GLTFIO_LIBRARIES}
      ${FILAMENT_LIBCXX_LIBRARIES}
      Threads::Threads
      ${CMAKE_DL_LIBS})
    message(STATUS "Found Filament: ${FILAMENT_DIR}")
  endif()
endif(FILAMENT_DIR)

if(NOT FILAMENT_FOUND AND NOT FILAMENT_DIR)
  message(STATUS "Filament not searched (set FILAMENT_DIR to the SDK root to enable the photorealism plugin)")
endif()
