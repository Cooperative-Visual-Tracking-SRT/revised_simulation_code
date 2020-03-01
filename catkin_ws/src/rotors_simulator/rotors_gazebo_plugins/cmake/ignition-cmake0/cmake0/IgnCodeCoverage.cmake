#
# 2012-01-31, Lars Bilke
# - Enable Code Coverage
#
# 2013-09-17, Joakim Söderberg
# - Added support for Clang.
# - Some additional usage instructions.
#
# 2017-09-13
# - Tweaked instructions for ignition libraries
# - Tweaked function name to avoid name collisions
#
# USAGE:
# 1. Add the following line to your CMakeLists.txt:
#      INCLUDE(IgnCodeCoverage)
#
# 2. Set compiler flags to turn off optimization and enable coverage:
#    SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
#    SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
#
# 3. Use the function IGN_SETUP_TARGET_FOR_COVERAGE to create a custom make target
#    which runs your test executable and produces a lcov code coverage report:
#    Example:
#    IGN_SETUP_TARGET_FOR_COVERAGE(
#        my_coverage_target  # Name for custom target.
#        test_driver         # Name of the test driver executable that runs the tests.
#                            # NOTE! This should always have a ZERO as exit code
#                            # otherwise the coverage generation will not complete.
#        coverage            # Name of output directory.
#        )
#
# 4. Build a Coverge build:
#   cmake -DCMAKE_BUILD_TYPE=Coverage ..
#   make
#   make my_coverage_target
#
#

# Check prereqs
FIND_PROGRAM( GCOV_PATH gcov )
FIND_PROGRAM( LCOV_PATH lcov )
FIND_PROGRAM( GREP_PATH grep )
FIND_PROGRAM( GENHTML_PATH genhtml )
FIND_PROGRAM( GCOVR_PATH gcovr PATHS ${CMAKE_SOURCE_DIR}/tests)

IF(NOT GCOV_PATH)
  MESSAGE(FATAL_ERROR "gcov not found! Aborting...")
ENDIF() # NOT GCOV_PATH

IF(NOT CMAKE_COMPILER_IS_GNUCXX)
  # Clang version 3.0.0 and greater now supports gcov as well.
  MESSAGE(WARNING "Compiler is not GNU gcc! Clang Version 3.0.0 and greater supports gcov as well, but older versions don't.")

  IF(NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    MESSAGE(FATAL_ERROR "Compiler is not GNU gcc! Aborting...")
  ENDIF()
ENDIF() # NOT CMAKE_COMPILER_IS_GNUCXX

IF ( NOT (CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "Coverage"))
  MESSAGE( WARNING "Code coverage results with an optimized (non-Debug) build may be misleading" )
ENDIF() # NOT CMAKE_BUILD_TYPE STREQUAL "Debug"


# Param _targetname     The name of new the custom make target
# Param _testrunner     The name of the target which runs the tests.
#						MUST return ZERO always, even on errors.
#						If not, no coverage report will be created!
# Param _outputname     lcov output is generated as _outputname.info
#                       HTML report is generated in _outputname/index.html
# Optional fourth parameter is passed as arguments to _testrunner
#   Pass them in list form, e.g.: "-j;2" for -j 2
#
# Code coverage is not run against files with the cxx extension (.cxx).
# We assume these files are created by swig.
FUNCTION(IGN_SETUP_TARGET_FOR_COVERAGE _targetname _testrunner _outputname)

  IF(NOT LCOV_PATH)
    MESSAGE(FATAL_ERROR "lcov not found! Aborting...")
  ENDIF() # NOT LCOV_PATH

  IF(NOT GREP_PATH)
    MESSAGE(FATAL_ERROR "grep not found! Run code coverage on linux or mac.")
  ENDIF()

  IF(NOT GENHTML_PATH)
    MESSAGE(FATAL_ERROR "genhtml not found! Aborting...")
  ENDIF() # NOT GENHTML_PATH

  # Setup target
  ADD_CUSTOM_TARGET(${_targetname}

    COMMAND ${CMAKE_COMMAND} -E remove ${_outputname}.info.cleaned
      ${_outputname}.info
    # Capturing lcov counters and generating report
    COMMAND ${LCOV_PATH} -q --no-checksum --directory ${PROJECT_BINARY_DIR}
      --capture --output-file ${_outputname}.info 2>/dev/null
    # Remove negative counts
    COMMAND sed -i '/,-/d' ${_outputname}.info
    COMMAND ${LCOV_PATH} -q --remove ${_outputname}.info
      'test/*' '/usr/*' '*_TEST*' '*.cxx' --output-file ${_outputname}.info.cleaned
    COMMAND ${GENHTML_PATH} -q --legend -o ${_outputname}
      ${_outputname}.info.cleaned
    COMMAND ${LCOV_PATH} --summary ${_outputname}.info.cleaned 2>&1 | grep "lines" | cut -d ' ' -f 4 | cut -d '%' -f 1 > coverage/lines.txt
    COMMAND ${LCOV_PATH} --summary ${_outputname}.info.cleaned 2>&1 | grep "functions" | cut -d ' ' -f 4 | cut -d '%' -f 1 > coverage/functions.txt
    COMMAND ${CMAKE_COMMAND} -E rename ${_outputname}.info.cleaned
      ${_outputname}.info

    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Resetting code coverage counters to zero.\n"
      "Processing code coverage counters and generating report."
  )

  # Show info where to find the report
  ADD_CUSTOM_COMMAND(TARGET ${_targetname} POST_BUILD
    COMMAND COMMAND ${LCOV_PATH} -q --zerocounters --directory ${PROJECT_BINARY_DIR};
    COMMENT "Open ./${_outputname}/index.html in your browser to view the coverage report."
  )

ENDFUNCTION() # IGN_SETUP_TARGET_FOR_COVERAGE
