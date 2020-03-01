#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ignition-msgs1::ignition-msgs1" for configuration "RelWithDebInfo"
set_property(TARGET ignition-msgs1::ignition-msgs1 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(ignition-msgs1::ignition-msgs1 PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libignition-msgs1.so.1.0.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libignition-msgs1.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS ignition-msgs1::ignition-msgs1 )
list(APPEND _IMPORT_CHECK_FILES_FOR_ignition-msgs1::ignition-msgs1 "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libignition-msgs1.so.1.0.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
