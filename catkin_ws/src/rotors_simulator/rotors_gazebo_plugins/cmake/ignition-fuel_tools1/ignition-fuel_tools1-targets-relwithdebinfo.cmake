#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ignition-fuel_tools1::ignition-fuel_tools1" for configuration "RelWithDebInfo"
set_property(TARGET ignition-fuel_tools1::ignition-fuel_tools1 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(ignition-fuel_tools1::ignition-fuel_tools1 PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libignition-fuel_tools1.so.1.0.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libignition-fuel_tools1.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS ignition-fuel_tools1::ignition-fuel_tools1 )
list(APPEND _IMPORT_CHECK_FILES_FOR_ignition-fuel_tools1::ignition-fuel_tools1 "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libignition-fuel_tools1.so.1.0.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
