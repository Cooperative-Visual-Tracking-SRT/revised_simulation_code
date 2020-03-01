# - Config file for the ignition-common1 package.
#
# For finding and loading ignition-common1 from your project, type:
#
# find_package(ignition-common1)
#
# It defines the following variables:
#
#  ignition-common1_FOUND        - System has ignition-common1.
#  ignition-common1_INCLUDE_DIRS - include directories for ignition-common1 and its dependencies.
#  ignition-common1_LIBRARY_DIRS - Paths in which the linker should search for libraries.
#  ignition-common1_LIBRARIES    - Libraries to link against.
#  ignition-common1_LDFLAGS      - Linker flags.
#
# Additionally, it will create an imported target named ignition-common1::ignition-common1.
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


if(ignition-common1_CONFIG_INCLUDED)
  return()
endif()
set(ignition-common1_CONFIG_INCLUDED TRUE)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was ignition-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../" ABSOLUTE)

# Use original install prefix when loaded through a "/usr move"
# cross-prefix symbolic link such as /lib -> /usr/lib.
get_filename_component(_realCurr "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
get_filename_component(_realOrig "/usr/lib/x86_64-linux-gnu/cmake/ignition-common1" REALPATH)
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

if(NOT TARGET ignition-common1::ignition-common1)
  include("${CMAKE_CURRENT_LIST_DIR}/ignition-common1-targets.cmake")
endif()

# On windows we produce .dll libraries with no prefix
if(WIN32)
 set(CMAKE_FIND_LIBRARY_PREFIXES "")
 set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib" ".dll")
endif()

# Package variables. Note that ignition-common1_LIBRARIES merely contains an imported
# target for the library, so it is sufficient to simply link to
# ignition-common1_LIBRARIES. None of the other package variables are needed.
set(ignition-common1_LIBRARIES ignition-common1::ignition-common1)
set(ignition-common1_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include/ignition/common1")
set(ignition-common1_LIBRARY_DIRS "${PACKAGE_PREFIX_DIR}/lib/x86_64-linux-gnu")
set(ignition-common1_LDFLAGS      "-L${PACKAGE_PREFIX_DIR}/lib/x86_64-linux-gnu")

# Backwards compatibility variables
set(IGNITION-COMMON_LIBRARIES ${ignition-common1_LIBRARIES})
set(IGNITION-COMMON_INCLUDE_DIRS ${ignition-common1_INCLUDE_DIRS})

# These variables are used by ignition-cmake to automatically configure the
# pkgconfig files for ignition projects.
set(ignition-common1_PKGCONFIG_ENTRY "ignition-common1")
set(ignition-common1_PKGCONFIG_TYPE PROJECT_PKGCONFIG_REQUIRES)

# Get access to the find_dependency utility
include(CMakeFindDependencyMacro)

# Find ignition-cmake, because we need its modules in order to find the rest of
# our dependencies.
find_dependency(ignition-cmake0)

# Find each dependency of this project (if nothing is below, then the project
# has no external dependencies).
find_dependency(ignition-math4)
find_dependency(TINYXML2)
find_dependency(DL)
find_dependency(FreeImage 3.9)
find_dependency(UUID)
find_dependency(GTS)
find_dependency(SWSCALE)
find_dependency(AVDEVICE 56.4.100)
find_dependency(AVFORMAT)
find_dependency(AVCODEC)
find_dependency(AVUTIL)

# Specify the doxygen tag file
set(IGNITION-COMMON_DOXYGEN_TAGFILE "/usr/share/ignition/ignition-common1_0/ignition-common1.tag.xml")

# Specify the API url. This is where the doxygen tag file will resolve URLS to.
set(IGNITION-COMMON_API_URL "https://ignitionrobotics.org/api/common/1.0")
