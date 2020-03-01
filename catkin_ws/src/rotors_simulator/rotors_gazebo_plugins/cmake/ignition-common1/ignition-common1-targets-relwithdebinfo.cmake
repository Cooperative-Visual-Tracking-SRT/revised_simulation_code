#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ignition-common1::ignition-common1" for configuration "RelWithDebInfo"
set_property(TARGET ignition-common1::ignition-common1 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(ignition-common1::ignition-common1 PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libignition-common1.so.1.0.1"
  IMPORTED_SONAME_RELWITHDEBINFO "libignition-common1.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS ignition-common1::ignition-common1 )
list(APPEND _IMPORT_CHECK_FILES_FOR_ignition-common1::ignition-common1 "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libignition-common1.so.1.0.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
