#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rviz_visual_testing_framework::rviz_visual_testing_framework" for configuration ""
set_property(TARGET rviz_visual_testing_framework::rviz_visual_testing_framework APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rviz_visual_testing_framework::rviz_visual_testing_framework PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librviz_visual_testing_framework.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS rviz_visual_testing_framework::rviz_visual_testing_framework )
list(APPEND _IMPORT_CHECK_FILES_FOR_rviz_visual_testing_framework::rviz_visual_testing_framework "${_IMPORT_PREFIX}/lib/librviz_visual_testing_framework.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
