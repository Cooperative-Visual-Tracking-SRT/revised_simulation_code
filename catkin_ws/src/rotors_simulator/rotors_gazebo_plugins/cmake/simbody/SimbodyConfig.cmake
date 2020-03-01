# SimbodyConfig.cmake

# Adapted from FindSimbody.cmake
#
# This should define the following:
#   Simbody_FOUND - Whether search for Simbody libraries and headers succeeded.
#   Simbody_ROOT_DIR - the installation directory; all the pieces must be
#                      found together
#   Simbody_INCLUDE_DIR - location of Simbody.h
#   Simbody_LIB_DIR     - location of libSimTKsimbody.{a,so,dylib} 
#                         or SimTKsimbody.lib
#   Simbody_BIN_DIR     - location of VisualizerGUI and .dll's on Windows
#   Simbody_LIBRARIES   - suitable for target_link_libraries(); includes
#                         both optimized and debug libraries if both are
#                         available
#   Simbody_STATIC_LIBRARIES - suitable for target_link_libraries(); includes
#                              both optimized and debug static libraries if
#                              both are available
#
# The following variables can be used in your own project so that your
# project's Doxygen documentation can link with Simbody's. These variables are
# only defined if Doxygen documentation is installed.
#   Simbody_DOXYGEN_DIR     - Directory containing Doxygen API documentation.
#   Simbody_DOXYGEN_TAGFILE - Path to SimbodyDoxygenTagFile.
#
# For example, if you're configuring your Doxyfile using CMake's
# configure_file, your Doxyfile.in file (to be configured) could contain
# (without the backslashes):
#
#   TAGFILES = "\@Simbody_DOXYGEN_TAGFILE\@=\@Simbody_DOXYGEN_DIR\@"

if (Simbody_CONFIG_INCLUDED)
  return()
endif()
set(Simbody_CONFIG_INCLUDED TRUE)

# Watch out for spaces in pathnames -- must quote.
list(APPEND Simbody_ROOT_DIR 
            "/usr")

list(APPEND Simbody_INCLUDE_DIR 
            "/usr/include/simbody")

list(APPEND Simbody_LIB_DIR 
            "/usr/lib/x86_64-linux-gnu")

list(APPEND Simbody_BIN_DIR 
            "/usr/libexec/simbody")

list(APPEND Simbody_CFLAGS 
            -I"/usr/include/simbody")

list(APPEND Simbody_LDFLAGS 
            -L"/usr/lib/x86_64-linux-gnu")

if (NOT "SimbodyDoxygenTagfile" STREQUAL "")
    # Must check tagfile variable, since the doxygen install dir is created
    # even if Doxygen documentation is not install.
    set(temp_doxygen_dir
        "/usr/share/doc/simbody/api")
    set(temp_tagfile_path
        "${temp_doxygen_dir}/SimbodyDoxygenTagfile")
    if (EXISTS "${temp_tagfile_path}")
        set(Simbody_DOXYGEN_DIR "${temp_doxygen_dir}")
        set(Simbody_DOXYGEN_TAGFILE "${temp_tagfile_path}")
    endif()
    unset(temp_doxygen_dir)
    unset(temp_tagfile_path)
endif()


# Find out which of the libraries are available.
find_library(Simbody_LIBRARY NAMES SimTKsimbody
    PATHS ${Simbody_LIB_DIR}
    DOC "This is the main Simbody library."
    NO_DEFAULT_PATH)
find_library(Simbody_STATIC_LIBRARY NAMES SimTKsimbody_static
    PATHS ${Simbody_LIB_DIR}
    DOC "This is the main Simbody static library."
    NO_DEFAULT_PATH)
find_library(Simbody_DEBUG_LIBRARY NAMES SimTKsimbody_d
    PATHS ${Simbody_LIB_DIR}
    DOC "This is the main Simbody debug library."
    NO_DEFAULT_PATH)
find_library(Simbody_STATIC_DEBUG_LIBRARY NAMES SimTKsimbody_static_d
    PATHS ${Simbody_LIB_DIR}
    DOC "This is the main Simbody static debug library."
    NO_DEFAULT_PATH)


# Set composite Simbody_LIBRARIES variable
set(LIBS)
if(Simbody_LIBRARY AND Simbody_DEBUG_LIBRARY)
  foreach(lib SimTKsimbody;SimTKmath;SimTKcommon)
    # must reset each time around the loop or find_library won't work
    set(onelib  "${lib}-NOTFOUND"   CACHE INTERNAL "nothing")
    set(onelibd "${lib}_d-NOTFOUND" CACHE INTERNAL "nothing")
    find_library(onelib ${lib}
      PATHS ${Simbody_LIB_DIR}
      NO_DEFAULT_PATH)
    find_library(onelibd ${lib}_d
      PATHS ${Simbody_LIB_DIR}
      NO_DEFAULT_PATH)
    if((NOT onelib) OR (NOT onelibd))
      message(FATAL_ERROR 
          "Library '${lib}' in package Simbody is not installed properly")
    endif()
    set(LIBS ${LIBS} optimized "${onelib}" debug "${onelibd}")
  endforeach()
elseif(Simbody_LIBRARY)
  foreach(lib SimTKsimbody;SimTKmath;SimTKcommon)
    set(onelib  "${lib}-NOTFOUND"   CACHE INTERNAL "nothing")
    find_library(onelib ${lib}
      PATHS ${Simbody_LIB_DIR}
      NO_DEFAULT_PATH)
    if(NOT onelib)
      message(FATAL_ERROR 
          "Library '${lib}' in package Simbody is not installed properly")
    endif()
    set(LIBS ${LIBS} "${onelib}")
  endforeach()
elseif(Simbody_DEBUG_LIBRARY)
  foreach(lib SimTKsimbody;SimTKmath;SimTKcommon)
    set(lib "${lib}_d")
    set(onelibd "${lib}-NOTFOUND" CACHE INTERNAL "nothing")
    find_library(onelibd ${lib}
      PATHS ${Simbody_LIB_DIR}
      NO_DEFAULT_PATH)
    if(NOT onelibd)
      message(FATAL_ERROR 
          "Library '${lib}' in package Simbody is not installed properly")
    endif()
    set(LIBS ${LIBS} "${onelibd}")
  endforeach()
endif()

if (LIBS)
    set(LIBS ${LIBS} "/usr/lib/x86_64-linux-gnu/libblas.so;/usr/lib/x86_64-linux-gnu/liblapack.so;/usr/lib/x86_64-linux-gnu/libblas.so;pthread;rt;dl;m")
    set(Simbody_LIBRARIES ${LIBS} CACHE STRING 
        "Simbody dynamic libraries" FORCE)
else()
    set(Simbody_LIBRARIES Simbody_LIBRARIES-NOTFOUND CACHE STRING 
        "Simbody dynamic libraries" FORCE)
endif()

# Static library
set(LIBS)
if(Simbody_STATIC_LIBRARY AND Simbody_STATIC_DEBUG_LIBRARY)
  foreach(lib SimTKsimbody;SimTKmath;SimTKcommon)
    set(lib "${lib}_static")
    set(onelib  "${lib}-NOTFOUND"   CACHE INTERNAL "nothing")
    set(onelibd "${lib}_d-NOTFOUND" CACHE INTERNAL "nothing")
    find_library(onelib ${lib}
      PATHS ${Simbody_LIB_DIR}
      NO_DEFAULT_PATH)
    find_library(onelibd ${lib}_d
      PATHS ${Simbody_LIB_DIR}
      NO_DEFAULT_PATH)
    if((NOT onelib) OR (NOT onelibd))
      message(FATAL_ERROR 
          "Library '${lib}' in package Simbody is not installed properly")
    endif()
    set(LIBS ${LIBS} optimized "${onelib}" debug "${onelibd}")
  endforeach()
elseif(Simbody_STATIC_LIBRARY)
  foreach(lib SimTKsimbody;SimTKmath;SimTKcommon)
    set(lib "${lib}_static")
    set(onelib  "${lib}-NOTFOUND"   CACHE INTERNAL "nothing")
    find_library(onelib ${lib}
      PATHS ${Simbody_LIB_DIR}
      NO_DEFAULT_PATH
      )
    if(NOT onelib)
      message(FATAL_ERROR 
          "Library '${lib}' in package Simbody is not installed properly")
    endif()
    set(LIBS ${LIBS} "${onelib}")
  endforeach()
elseif(Simbody_STATIC_DEBUG_LIBRARY)
  foreach(lib SimTKsimbody;SimTKmath;SimTKcommon)
    set(lib "${lib}_static_d")
    set(onelibd  "${lib}-NOTFOUND"   CACHE INTERNAL "nothing")
    find_library(onelibd ${lib}
      PATHS ${Simbody_LIB_DIR}
      NO_DEFAULT_PATH
      )
    if(NOT onelibd)
      message(FATAL_ERROR 
          "Library '${lib}' in package Simbody is not installed properly")
    endif()
    set(LIBS ${LIBS} "${onelibd}")
  endforeach()
endif()

if (LIBS)
    # these aren't available in static
    set(LIBS ${LIBS} "/usr/lib/x86_64-linux-gnu/libblas.so;/usr/lib/x86_64-linux-gnu/liblapack.so;/usr/lib/x86_64-linux-gnu/libblas.so;pthread;rt;dl;m")
    set(Simbody_STATIC_LIBRARIES "${LIBS}" CACHE STRING 
        "Simbody static libraries" FORCE)
else()
    set(Simbody_STATIC_LIBRARIES Simbody_STATIC_LIBRARIES-NOTFOUND CACHE STRING 
        "Simbody static libraries" FORCE)
endif()

# This CMake-supplied script provides standard error handling.
include(FindPackageHandleStandardArgs OPTIONAL)
find_package_handle_standard_args(Simbody DEFAULT_MSG 
    Simbody_INCLUDE_DIR)

# Not all the variables we produced need be returned.
unset(onelib CACHE)
unset(onelibd CACHE)
unset(Simbody_LIBRARY CACHE)
unset(Simbody_DEBUG_LIBRARY CACHE)
unset(Simbody_STATIC_LIBRARY CACHE)
unset(Simbody_STATIC_DEBUG_LIBRARY CACHE)
mark_as_advanced(Simbody_LIBRARIES Simbody_STATIC_LIBRARIES)
