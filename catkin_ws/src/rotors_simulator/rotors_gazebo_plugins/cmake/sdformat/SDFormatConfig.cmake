if (SDFormat_CONFIG_INCLUDED)
  return()
endif()
set(SDFormat_CONFIG_INCLUDED TRUE)

list(APPEND SDFormat_INCLUDE_DIRS "/usr/include/sdformat-6.0")

list(APPEND SDFormat_CFLAGS "-I/usr/include/sdformat-6.0")
if (NOT WIN32)
  list(APPEND SDFormat_CXX_FLAGS "${SDFormat_CFLAGS} -std=c++11")
endif()

list(APPEND SDFormat_LIBRARY_DIRS "/usr/lib/x86_64-linux-gnu")

foreach(lib sdformat)
  set(onelib "${lib}-NOTFOUND")
  find_library(onelib ${lib}
    PATHS "/usr/lib/x86_64-linux-gnu"
    NO_DEFAULT_PATH
    )
  if(NOT onelib)
    message(FATAL_ERROR "Library '${lib}' in package SDFormat is not installed properly")
  endif()
  list(APPEND SDFormat_LIBRARIES "${onelib}")
endforeach()

find_package(ignition-math4)
list(APPEND SDFormat_INCLUDE_DIRS ${IGNITION-MATH_INCLUDE_DIRS})
list(APPEND SDFormat_LIBRARIES ${IGNITION-MATH_LIBRARIES})
list(APPEND SDFormat_LIBRARY_DIRS ${IGNITION-MATH_LIBRARY_DIRS})

find_package(Boost)
list(APPEND SDFormat_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})

list(APPEND SDFormat_LDFLAGS "-L/usr/lib/x86_64-linux-gnu")
