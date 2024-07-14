# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_nsc_core_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED nsc_core_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(nsc_core_FOUND FALSE)
  elseif(NOT nsc_core_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(nsc_core_FOUND FALSE)
  endif()
  return()
endif()
set(_nsc_core_CONFIG_INCLUDED TRUE)

# output package information
if(NOT nsc_core_FIND_QUIETLY)
  message(STATUS "Found nsc_core: 0.0.0 (${nsc_core_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'nsc_core' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${nsc_core_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(nsc_core_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${nsc_core_DIR}/${_extra}")
endforeach()
