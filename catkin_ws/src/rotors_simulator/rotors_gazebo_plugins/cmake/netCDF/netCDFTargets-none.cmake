#----------------------------------------------------------------
# Generated CMake target import file for configuration "NONE".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "netcdf" for configuration "NONE"
set_property(TARGET netcdf APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(netcdf PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libnetcdf.so.13"
  IMPORTED_SONAME_NONE "libnetcdf.so.13"
  )

list(APPEND _IMPORT_CHECK_TARGETS netcdf )
list(APPEND _IMPORT_CHECK_FILES_FOR_netcdf "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libnetcdf.so.13" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
