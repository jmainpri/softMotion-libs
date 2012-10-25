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

CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

PROJECT (SoftMotionPlanner)

#Default mode is Debug
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

	
# Set the project version

set( PROJECT_MAJOR_VERSION 3)
set( PROJECT_MINOR_VERSION 11)

set( PROJECT_VERSION "${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}")
set(SOFTMOTION_VERSION "${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}")

# Define the project version
if( NOT NON_NATIVE_TARGET)
add_definitions( -DPROJECT_VERSION="${PROJECT_VERSION}")
endif( NOT NON_NATIVE_TARGET)

# CMake information
message( STATUS "CMake System = ${CMAKE_SYSTEM_NAME}")
message( STATUS "CMake Processor = ${CMAKE_SYSTEM_PROCESSOR}")
message( STATUS "CMake Source Dir = ${CMAKE_SOURCE_DIR}")
message( STATUS "CMake Binary Dir = ${CMAKE_BINARY_DIR}")
message( STATUS "CMake Install Prefix = ${CMAKE_INSTALL_PREFIX}")
	

# -------------------------------------------------------------
# --- Default compilation option : ----------------------------
# --- DO NOT MODIFY!!! use ccmake -----------------------------
# -------------------------------------------------------------
SET(SMP_CompilationFlags "")
SET(SMP_LinkedCompilationFlags "")

MACRO(SMP_flag flagName description status)
  SET(flagName "${flagName}")
  SET(description "${description}")
  SET(status "${status}")
  OPTION(${flagName} ${description} ${status})
  LIST(APPEND SMP_CompilationFlags ${flagName})
  FOREACH(linkFlag ${ARGN})
    LIST(APPEND SMP_LinkedCompilationFlags "${flagName}:${linkFlag}")
  ENDFOREACH(linkFlag)
#  set_property(CACHE ANIMATION PROPERTY VALUE ON)
ENDMACRO(SMP_flag)

MACRO(SMP_CheckCompilationFlags)
  SET(SMP_ActiveFlags "")
  MESSAGE ("Active Flags :")
  FOREACH(flag ${SMP_CompilationFlags})
    IF(${flag})
      FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/smpFlags "-D${flag} ")
      MESSAGE("${flag}")
      ADD_DEFINITIONS("-D${flag}")
      LIST(APPEND SMP_ActiveFlags ${flag})
    ENDIF()
  ENDFOREACH(flag)

ENDMACRO(SMP_CheckCompilationFlags)


SMP_flag(ENABLE_DISPLAY "Sets the Qt interface active" OFF)
SMP_flag(QT_LIBRARY "Sets QtLibrary compiling mode" OFF)
SMP_flag(WITH_TCL "Sets the tcl package compiling mode" OFF)

SMP_CheckCompilationFlags()

add_subdirectory(src)
add_subdirectory(pkgconfig)
IF(WITH_TCL)
add_subdirectory(tclsrc)
add_subdirectory(tcl)
ENDIF(WITH_TCL)
IF(QT_LIBRARY)
add_subdirectory(qt)
ENDIF(QT_LIBRARY)


MACRO(GENERATE_DOCUMENTATION DOXYGEN_CONFIG_FILE)
FIND_PACKAGE(Doxygen)
SET(DOXYFILE_FOUND false)
IF(EXISTS ${PROJECT_SOURCE_DIR}/${DOXYGEN_CONFIG_FILE})
    SET(DOXYFILE_FOUND true)
ENDIF(EXISTS ${PROJECT_SOURCE_DIR}/${DOXYGEN_CONFIG_FILE})

IF( DOXYGEN_FOUND )
    IF( DOXYFILE_FOUND )
        # Add target

ADD_CUSTOM_TARGET( doc ALL ${DOXYGEN_EXECUTABLE} "${PROJECT_SOURCE_DIR}/${DOXYGEN_CONFIG_FILE}" )


    ELSE( DOXYFILE_FOUND )
MESSAGE( STATUS "Doxygen configuration file not found - Documentation will not be generated" )
    ENDIF( DOXYFILE_FOUND )
ELSE(DOXYGEN_FOUND)
    MESSAGE(STATUS "Doxygen not found - Documentation will not be generated")
ENDIF(DOXYGEN_FOUND)
ENDMACRO(GENERATE_DOCUMENTATION)


GENERATE_DOCUMENTATION(Doxyfile)
