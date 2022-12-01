# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_imperial_driverless_utils_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED imperial_driverless_utils_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(imperial_driverless_utils_FOUND FALSE)
  elseif(NOT imperial_driverless_utils_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(imperial_driverless_utils_FOUND FALSE)
  endif()
  return()
endif()
set(_imperial_driverless_utils_CONFIG_INCLUDED TRUE)

# output package information
if(NOT imperial_driverless_utils_FIND_QUIETLY)
  message(STATUS "Found imperial_driverless_utils: 0.13.1 (${imperial_driverless_utils_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'imperial_driverless_utils' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${imperial_driverless_utils_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(imperial_driverless_utils_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${imperial_driverless_utils_DIR}/${_extra}")
endforeach()
