#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rclcpp_components::component_manager" for configuration ""
set_property(TARGET rclcpp_components::component_manager APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rclcpp_components::component_manager PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcomponent_manager.so"
  IMPORTED_SONAME_NOCONFIG "libcomponent_manager.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rclcpp_components::component_manager )
list(APPEND _IMPORT_CHECK_FILES_FOR_rclcpp_components::component_manager "${_IMPORT_PREFIX}/lib/libcomponent_manager.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
