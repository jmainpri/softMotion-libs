#
# Copyright (c) 2010,2013 LAAS/CNRS
# All rights reserved.
#
# Permission to use, copy, modify, and distribute this software for any purpose
# with or without   fee is hereby granted, provided   that the above  copyright
# notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS  SOFTWARE INCLUDING ALL  IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS. IN NO EVENT SHALL THE AUTHOR  BE LIABLE FOR ANY SPECIAL, DIRECT,
# INDIRECT, OR CONSEQUENTIAL DAMAGES OR  ANY DAMAGES WHATSOEVER RESULTING  FROM
# LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
# OTHER TORTIOUS ACTION,   ARISING OUT OF OR IN    CONNECTION WITH THE USE   OR
# PERFORMANCE OF THIS SOFTWARE.
#
#                                            Xavier BROQUERE on Fri Feb 26 2010

project(src CXX)

set(source_files
  Sm_Approx.cpp
  Sm_Curve.cpp
  Sm_Traj.cpp
  debug.c
  gnuplot_i.cpp
  matrix.c
  softMotion.c
  time.c)
set_source_files_properties(${source_files} PROPERTIES LANGUAGE CXX)

FIND_PACKAGE(LibXml2 REQUIRED)
INCLUDE_directories(${LIBXML2_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})
SET(LIBS ${LIBS} ${LIBXML2_LIBRARIES})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
#INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/qt)

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})


IF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  ADD_DEFINITIONS(-g -Wall -Wno-unknown-pragmas)
  ADD_DEFINITIONS(-g -Wall -Wno-unknown-pragmas -DLOGLEVEL=0)
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  ADD_DEFINITIONS(-O2 -Wall -Wno-unknown-pragmas)
ENDIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")


#ADD_DEFINITIONS(-Wno-unused-but-set-variable)

add_library(
softMotion
SHARED
${source_files}
)

add_executable(svg2traj svg2traj.cpp)


#add_executable(
#softMotionPlannerTest
#${source_files}
#)

TARGET_LINK_LIBRARIES(softMotion ${LIBS} m)
TARGET_LINK_LIBRARIES(svg2traj softMotion)

#TARGET_LINK_LIBRARIES(softMotionPlannerTest ${LIBS})

SET_TARGET_PROPERTIES(softMotion PROPERTIES LINKER_LANGUAGE CXX)

# -------------------------------------------------------------
# --- Installation --------------------------------------------
# -------------------------------------------------------------

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "SoftMotionPlanner install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
INSTALL(TARGETS softMotion

        LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ CONFIGURATIONS ${CMAKE_BUILD_TYPE})

INSTALL(TARGETS svg2traj RUNTIME DESTINATION bin)

FILE(GLOB HEADER "${CMAKE_SOURCE_DIR}/src/*.h"  "${CMAKE_SOURCE_DIR}/src/*.hpp" )
INSTALL(FILES ${HEADER} DESTINATION include/softMotion)
