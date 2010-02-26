CMAKE_MINIMUM_REQUIRED (VERSION 2.6.4)

PROJECT (SoftMotionPlanner)

#Default mode is Debug
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)



add_subdirectory(src)
add_subdirectory(tclsrc)
add_subdirectory(tcl)
