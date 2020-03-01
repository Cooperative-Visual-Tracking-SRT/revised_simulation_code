# - Config file for ignition-cmake (not for any other ignition libraries).
#
# To find and load ignition-cmake modules into your project, simply invoke:
#
# find_package(ignition-cmake0)
#
# That will define the variable ignition-cmake0_FOUND, and will open up access
# to all the cmake-modules and find-modules that are provided by ignition-cmake.

# We explicitly set the desired cmake version to ensure that the policy settings
# of users or of toolchains do not result in the wrong behavior for our modules.
# Note that the call to find_package(~) will PUSH a new policy stack before
# taking on these version settings, and then that stack will POP after the
# find_package(~) has exited, so this will not affect the cmake policy settings
# of a caller.
cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#--------------------------------------
# Initialize the IGNITION_CMAKE_DIR variable with the location of the cmake
# directory that sits next to this find-module.
set(IGNITION_CMAKE_DIR "${CMAKE_CURRENT_LIST_DIR}/cmake0")

#--------------------------------------
# Add the location of this package's cmake directory to the CMAKE_MODULE_PATH
list(APPEND CMAKE_MODULE_PATH "${IGNITION_CMAKE_DIR}")

#--------------------------------------
# include the master IgnCMake module
include(IgnCMake)

#--------------------------------------
# Create a variable to indicate what version of ignition-cmake we are using.
# This variable does not follow the usual cmake naming convention because it is
# a non-standard package variable.
set(IGNITION_CMAKE_VERSION_MAJOR 0)

set(IGNITION_CMAKE_DOXYGEN_DIR "/usr/share/ignition/ignition-cmake0/doxygen")
