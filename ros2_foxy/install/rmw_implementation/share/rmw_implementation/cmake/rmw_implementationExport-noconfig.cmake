#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rmw_implementation::rmw_implementation" for configuration ""
set_property(TARGET rmw_implementation::rmw_implementation APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rmw_implementation::rmw_implementation PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librmw_implementation.so"
  IMPORTED_SONAME_NOCONFIG "librmw_implementation.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rmw_implementation::rmw_implementation )
list(APPEND _IMPORT_CHECK_FILES_FOR_rmw_implementation::rmw_implementation "${_IMPORT_PREFIX}/lib/librmw_implementation.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
