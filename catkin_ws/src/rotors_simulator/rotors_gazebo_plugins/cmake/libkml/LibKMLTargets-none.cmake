#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "kmlbase" for configuration "None"
set_property(TARGET kmlbase APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(kmlbase PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NONE "/usr/lib/x86_64-linux-gnu/libexpat.so;/usr/lib/x86_64-linux-gnu/libz.so;/usr/lib/x86_64-linux-gnu/libminizip.so;/usr/lib/x86_64-linux-gnu/liburiparser.so;/usr/lib/x86_64-linux-gnu/libexpat.so"
  IMPORTED_LOCATION_NONE "/usr/lib/x86_64-linux-gnu/libkmlbase.so.1.3.0"
  IMPORTED_SONAME_NONE "libkmlbase.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS kmlbase )
list(APPEND _IMPORT_CHECK_FILES_FOR_kmlbase "/usr/lib/x86_64-linux-gnu/libkmlbase.so.1.3.0" )

# Import target "kmldom" for configuration "None"
set_property(TARGET kmldom APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(kmldom PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NONE "kmlbase"
  IMPORTED_LOCATION_NONE "/usr/lib/x86_64-linux-gnu/libkmldom.so.1.3.0"
  IMPORTED_SONAME_NONE "libkmldom.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS kmldom )
list(APPEND _IMPORT_CHECK_FILES_FOR_kmldom "/usr/lib/x86_64-linux-gnu/libkmldom.so.1.3.0" )

# Import target "kmlxsd" for configuration "None"
set_property(TARGET kmlxsd APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(kmlxsd PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NONE "kmlbase"
  IMPORTED_LOCATION_NONE "/usr/lib/x86_64-linux-gnu/libkmlxsd.so.1.3.0"
  IMPORTED_SONAME_NONE "libkmlxsd.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS kmlxsd )
list(APPEND _IMPORT_CHECK_FILES_FOR_kmlxsd "/usr/lib/x86_64-linux-gnu/libkmlxsd.so.1.3.0" )

# Import target "kmlengine" for configuration "None"
set_property(TARGET kmlengine APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(kmlengine PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NONE "kmlbase;kmldom"
  IMPORTED_LOCATION_NONE "/usr/lib/x86_64-linux-gnu/libkmlengine.so.1.3.0"
  IMPORTED_SONAME_NONE "libkmlengine.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS kmlengine )
list(APPEND _IMPORT_CHECK_FILES_FOR_kmlengine "/usr/lib/x86_64-linux-gnu/libkmlengine.so.1.3.0" )

# Import target "kmlconvenience" for configuration "None"
set_property(TARGET kmlconvenience APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(kmlconvenience PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NONE "kmlengine;kmldom;kmlbase"
  IMPORTED_LOCATION_NONE "/usr/lib/x86_64-linux-gnu/libkmlconvenience.so.1.3.0"
  IMPORTED_SONAME_NONE "libkmlconvenience.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS kmlconvenience )
list(APPEND _IMPORT_CHECK_FILES_FOR_kmlconvenience "/usr/lib/x86_64-linux-gnu/libkmlconvenience.so.1.3.0" )

# Import target "kmlregionator" for configuration "None"
set_property(TARGET kmlregionator APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(kmlregionator PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NONE "kmlbase;kmldom;kmlengine;kmlconvenience"
  IMPORTED_LOCATION_NONE "/usr/lib/x86_64-linux-gnu/libkmlregionator.so.1.3.0"
  IMPORTED_SONAME_NONE "libkmlregionator.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS kmlregionator )
list(APPEND _IMPORT_CHECK_FILES_FOR_kmlregionator "/usr/lib/x86_64-linux-gnu/libkmlregionator.so.1.3.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
