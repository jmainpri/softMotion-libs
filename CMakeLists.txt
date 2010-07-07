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

CMAKE_MINIMUM_REQUIRED (VERSION 2.6.4)

PROJECT (SoftMotionPlanner)

#Default mode is Debug
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

	
# Set the project version

if( NOT PROJECT_VERSION)


#if( NOT PROJECT_WC_REVISION)
#message( "Unable to determine subversion revision")
#set( PROJECT_WC_REVISION 1)
#endif( NOT PROJECT_WC_REVISION)

if( NOT PROJECT_MAJOR_VERSION)
#message( STATUS "PROJECT_MAJOR_VERSION is not set")
set( PROJECT_MAJOR_VERSION 1)
endif( NOT PROJECT_MAJOR_VERSION)

if( NOT PROJECT_MINOR_VERSION)
#message( STATUS "PROJECT_MINOR_VERSION is not set")
set( PROJECT_MINOR_VERSION 0)
endif( NOT PROJECT_MINOR_VERSION)

#message( STATUS "Current subversion revision is ${PROJECT_WC_REVISION}")

#set( PROJECT_REVISION ${PROJECT_WC_REVISION})
set( PROJECT_VERSION "${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}")

endif( NOT PROJECT_VERSION)

if( NOT DEFINED PROJECT_MAJOR_VERSION)
message( FATAL_ERROR "The project major version has not been defined.")
endif( NOT DEFINED PROJECT_MAJOR_VERSION)
	
if( NOT DEFINED PROJECT_MINOR_VERSION)
message( FATAL_ERROR "The project minor version has not been defined.")
endif( NOT DEFINED PROJECT_MINOR_VERSION)

#if( NOT DEFINED PROJECT_REVISION)
#message( FATAL_ERROR "The project revision has not been defined.")
#endif( NOT DEFINED PROJECT_REVISION)

#message( STATUS "Current project major version is ${PROJECT_MAJOR_VERSION}")
#message( STATUS "Current project minor version is ${PROJECT_MINOR_VERSION}")
#message( STATUS "Current project revision is ${PROJECT_REVISION}")
message( STATUS "Current project version is ${PROJECT_VERSION}")

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
  FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/smpFlags "#Generated File Do not modify!!!\n" )
  FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/smpFlags "SOFTMOTIONPLANNER_COMPILATION_FLAGS = ")
  FOREACH(flag ${SMP_CompilationFlags})
    IF(${flag})
      FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/smpFlags "-D${flag} ")
      MESSAGE("${flag}")
      ADD_DEFINITIONS("-D${flag}")
      LIST(APPEND SMP_ActiveFlags ${flag})
    ENDIF()
  ENDFOREACH(flag)

#  MESSAGE ("\nLinked Flags :")
#  FOREACH(activeFlag ${SMP_ActiveFlags})
#    FOREACH(linkFlag ${SMP_LinkedCompilationFlags})
#      IF(${linkFlag} MATCHES "${activeFlag}:(.*)")
#        STRING(LENGTH "${activeFlag}:" prefixLength)
#        STRING(LENGTH ${linkFlag} linkLength)
#        MATH(EXPR linkLength "${linkLength} - ${prefixLength}")
#        STRING(SUBSTRING ${linkFlag} ${prefixLength} ${linkLength} linkFlagToActivate)
#        SET(isAlreadyActive 0)
#        FOREACH(activeFlag2 ${SMP_ActiveFlags})
#          IF(${activeFlag2} STREQUAL ${linkFlagToActivate})
#            SET(isAlreadyActive 1)
#          ENDIF(${activeFlag2} STREQUAL ${linkFlagToActivate})
#        ENDFOREACH(activeFlag2)
#        IF(${isAlreadyActive} EQUAL 0)
#          SET(${linkFlagToActivate} ON)
#
#          MESSAGE("${activeFlag} -> ${linkFlagToActivate}")
#          ADD_DEFINITIONS("-D${linkFlagToActivate}")
#        ENDIF(${isAlreadyActive} EQUAL 0)
#      ENDIF(${linkFlag} MATCHES "${activeFlag}:(.*)")
#    ENDFOREACH(linkFlag)
#  ENDFOREACH(activeFlag)
#  MESSAGE("\n")
ENDMACRO(SMP_CheckCompilationFlags)

SMP_flag(ENABLE_DISPLAY "Sets the Qt interface active" OFF)
SMP_flag(QT_LIBRARY "Sets QtLibrary compiling mode" OFF)
SMP_flag(WITH_TCL "Sets the tcl package compiling mode" OFF)

SMP_CheckCompilationFlags()


add_subdirectory(src)
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
