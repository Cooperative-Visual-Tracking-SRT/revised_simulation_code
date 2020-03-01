#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ignition-math4::ignition-math4" for configuration "RelWithDebInfo"
set_property(TARGET ignition-math4::ignition-math4 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(ignition-math4::ignition-math4 PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libignition-math4.so.4.0.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libignition-math4.so.4"
  )

list(APPEND _IMPORT_CHECK_TARGETS ignition-math4::ignition-math4 )
list(APPEND _IMPORT_CHECK_FILES_FOR_ignition-math4::ignition-math4 "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libignition-math4.so.4.0.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
