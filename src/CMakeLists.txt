#
# Copyright (c) 2010 LAAS/CNRS
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

project(src)

file(GLOB source_files ./*.c ./*.cpp)



SET(CMAKE_CXX_FLAGS " -lm ")

SET(CMAKE_C_COMPILER g++)
SET(CMAKE_CXX_COMPILER g++)

FIND_PACKAGE(LibXml2 REQUIRED)
INCLUDE_directories(${LIBXML2_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})
SET(LIBS ${LIBS} ${LIBXML2_LIBRARIES})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
#INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/qt)

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})


IF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  ADD_DEFINITIONS(-g -Wall -Wno-unknown-pragmas)
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  ADD_DEFINITIONS(-O2 -Wall -Wno-unknown-pragmas)
ENDIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")


add_library(
softMotion
SHARED
${source_files}
)

#add_executable(
#softMotionPlannerTest
#${source_files}
#)
IF(APPLE)
TARGET_LINK_LIBRARIES(softMotion ${LIBS})
ENDIF(APPLE)
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

FILE(GLOB HEADER "${CMAKE_SOURCE_DIR}/src/*.h")
INSTALL(FILES ${HEADER} DESTINATION include/softMotion)


