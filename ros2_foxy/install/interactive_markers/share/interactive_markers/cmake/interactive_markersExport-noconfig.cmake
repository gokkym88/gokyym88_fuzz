#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "interactive_markers::interactive_markers" for configuration ""
set_property(TARGET interactive_markers::interactive_markers APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(interactive_markers::interactive_markers PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libinteractive_markers.so"
  IMPORTED_SONAME_NOCONFIG "libinteractive_markers.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS interactive_markers::interactive_markers )
list(APPEND _IMPORT_CHECK_FILES_FOR_interactive_markers::interactive_markers "${_IMPORT_PREFIX}/lib/libinteractive_markers.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
