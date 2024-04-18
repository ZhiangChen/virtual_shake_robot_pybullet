# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_virtual_shake_robot_pybullet_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED virtual_shake_robot_pybullet_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(virtual_shake_robot_pybullet_FOUND FALSE)
  elseif(NOT virtual_shake_robot_pybullet_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(virtual_shake_robot_pybullet_FOUND FALSE)
  endif()
  return()
endif()
set(_virtual_shake_robot_pybullet_CONFIG_INCLUDED TRUE)

# output package information
if(NOT virtual_shake_robot_pybullet_FIND_QUIETLY)
  message(STATUS "Found virtual_shake_robot_pybullet: 0.0.0 (${virtual_shake_robot_pybullet_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'virtual_shake_robot_pybullet' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${virtual_shake_robot_pybullet_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(virtual_shake_robot_pybullet_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${virtual_shake_robot_pybullet_DIR}/${_extra}")
endforeach()
