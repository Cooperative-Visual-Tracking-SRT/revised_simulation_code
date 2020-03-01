# - Config file for the ignition-transport4 package.
#
# For finding and loading ignition-transport4 from your project, type:
#
# find_package(ignition-transport4)
#
# It defines the following variables:
#
#  ignition-transport4_FOUND        - System has ignition-transport4.
#  ignition-transport4_INCLUDE_DIRS - include directories for ignition-transport4 and its dependencies.
#  ignition-transport4_LIBRARY_DIRS - Paths in which the linker should search for libraries.
#  ignition-transport4_LIBRARIES    - Libraries to link against.
#  ignition-transport4_LDFLAGS      - Linker flags.
#
# Additionally, it will create an imported target named ignition-transport4::ignition-transport4.
# You can link your library against that target using target_link_library(~),
# and all the variables mentioned above will automatically be pulled into your
# target.

# We explicitly set the desired cmake version to ensure that the policy settings
# of users or of toolchains do not result in the wrong behavior for our modules.
# Note that the call to find_package(~) will PUSH a new policy stack before
# taking on these version settings, and then that stack will POP after the
# find_package(~) has exited, so this will not affect the cmake policy settings
# of a caller.
cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)


if(ignition-transport4_CONFIG_INCLUDED)
  return()
endif()
set(ignition-transport4_CONFIG_INCLUDED TRUE)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was ignition-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../" ABSOLUTE)

# Use original install prefix when loaded through a "/usr move"
# cross-prefix symbolic link such as /lib -> /usr/lib.
get_filename_component(_realCurr "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
get_filename_component(_realOrig "/usr/lib/x86_64-linux-gnu/cmake/ignition-transport4" REALPATH)
if(_realCurr STREQUAL _realOrig)
  set(PACKAGE_PREFIX_DIR "/usr")
endif()
unset(_realOrig)
unset(_realCurr)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

if(NOT TARGET ignition-transport4::ignition-transport4)
  include("${CMAKE_CURRENT_LIST_DIR}/ignition-transport4-targets.cmake")
endif()

# On windows we produce .dll libraries with no prefix
if(WIN32)
 set(CMAKE_FIND_LIBRARY_PREFIXES "")
 set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib" ".dll")
endif()

# Package variables. Note that ignition-transport4_LIBRARIES merely contains an imported
# target for the library, so it is sufficient to simply link to
# ignition-transport4_LIBRARIES. None of the other package variables are needed.
set(ignition-transport4_LIBRARIES ignition-transport4::ignition-transport4)
set(ignition-transport4_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include/ignition/transport4")
set(ignition-transport4_LIBRARY_DIRS "${PACKAGE_PREFIX_DIR}/lib/x86_64-linux-gnu")
set(ignition-transport4_LDFLAGS      "-L${PACKAGE_PREFIX_DIR}/lib/x86_64-linux-gnu")

# Backwards compatibility variables
set(IGNITION-TRANSPORT_LIBRARIES ${ignition-transport4_LIBRARIES})
set(IGNITION-TRANSPORT_INCLUDE_DIRS ${ignition-transport4_INCLUDE_DIRS})

# These variables are used by ignition-cmake to automatically configure the
# pkgconfig files for ignition projects.
set(ignition-transport4_PKGCONFIG_ENTRY "ignition-transport4")
set(ignition-transport4_PKGCONFIG_TYPE PROJECT_PKGCONFIG_REQUIRES)

# Get access to the find_dependency utility
include(CMakeFindDependencyMacro)

# Find ignition-cmake, because we need its modules in order to find the rest of
# our dependencies.
find_dependency(ignition-cmake0)

# Find each dependency of this project (if nothing is below, then the project
# has no external dependencies).
find_dependency(IgnProtobuf 2.3.0)
find_dependency(ZeroMQ 4)
find_dependency(CPPZMQ)
find_dependency(UUID)
find_dependency(ignition-msgs1)

# Specify the doxygen tag file
set(IGNITION-TRANSPORT_DOXYGEN_TAGFILE "/usr/share/ignition/ignition-transport4_0/ignition-transport4.tag.xml")

# Specify the API url. This is where the doxygen tag file will resolve URLS to.
set(IGNITION-TRANSPORT_API_URL "https://ignitionrobotics.org/api/transport/4.0")
