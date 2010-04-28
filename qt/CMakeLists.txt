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


PROJECT (QSoftMotionPlanner C CXX)

file(GLOB source_files ./*.cpp)

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


SMP_flag(QT_LIBRARY "Sets QtLibrary compiling mode" ON)

SMP_CheckCompilationFlags()

#################################################################
#### Add here a library to include (custom FindPackage.cmake ####
#### are in SoftMotionPlanner/.cmakeScripts. Use the template to     ####
####Â create a custom one)                                    ####
#################################################################
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/script/CMake")
IF(QT_LIBRARY)

  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE_DIRECTORIES(${QT_INCLUDE_DIR})
  SET(SOFTMOTIONPLANNER_COMPILATION_INCLUDES ${SOFTMOTIONPLANNER_COMPILATION_INCLUDES} ${QT_INCLUDE_DIR})


  SET(QT_USE_QTOPENGL TRUE)
  INCLUDE(${QT_USE_FILE})
  SET(LIBS ${LIBS} ${QT_LIBRARIES})


  FIND_PACKAGE(QXML REQUIRED)
  INCLUDE_directories(${QXML_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${QXML_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${QXML_LIBRARIES})

  FIND_PACKAGE(QWT REQUIRED)
  INCLUDE_directories(${QWT_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${QWT_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${QWT_LIBRARIES})

  FIND_PACKAGE(QGLVIEWER REQUIRED)
  INCLUDE_directories(${QGLVIEWER_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${QGLVIEWER_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${QGLVIEWER_LIBRARIES})


FIND_PACKAGE(LibXml2 REQUIRED)
INCLUDE_directories(${LIBXML2_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})
SET(LIBS ${LIBS} ${LIBXML2_LIBRARIES})



#####################################################################
#### Add here a src directory containing a SourceList.cmake file ####
#####################################################################

MACRO(BM3D_SRC_SUBDIR_PROCESS)
  FOREACH(srcfile ${ARGV})
    LIST(APPEND BM3D_src "${BM3D_MODULE_NAME}/${srcfile}")
    SOURCE_GROUP(${BM3D_MODULE_NAME} FILES ${BM3D_MODULE_NAME}/${srcfile})
  ENDFOREACH(srcfile)
ENDMACRO(BM3D_SRC_SUBDIR_PROCESS)
MACRO(BM3D_INC_DIR_PROCESS directory)
  include_directories (${directory})
  file (GLOB incfiles "${directory}/*.h")
  LIST(APPEND BM3D_inc "${incfiles}")
  SOURCE_GROUP(${BM3D_MODULE_NAME} FILES ${incfiles})
  file (GLOB incfiles "${directory}/*.hpp")
  LIST(APPEND BM3D_inc "${incfiles}")
  SOURCE_GROUP(${BM3D_MODULE_NAME} FILES ${incfiles})
ENDMACRO(BM3D_INC_DIR_PROCESS)

# -------------------------------------------------------------
# --- QT4 SUBDIR PROCESS (same but for preprocessing qt4 files)
# -------------------------------------------------------------
SET(SMP_QT_FILES_FOR_MOC "")
MACRO(SMP_QT_GENERATE_MOC)
  FOREACH(file ${ARGV})
    LIST(APPEND SMP_QT_FILES_FOR_MOC "${file}")
  ENDFOREACH(file ${ARGV})
ENDMACRO(SMP_QT_GENERATE_MOC)

SET(SMP_QT_UI_FILES "")
MACRO(SMP_QT_GENERATE_UI_HEADERS)
  FOREACH(file ${ARGV})
    LIST(APPEND SMP_QT_UI_FILES "${file}")
  ENDFOREACH(file ${ARGV})
ENDMACRO(SMP_QT_GENERATE_UI_HEADERS)


#MESSAGE("bm3dsrc : ${BM3D_src}") #Uncomment to print the included file by Cmake

# -------------------------------------------------------------
# --- Directories to include ----------------------------------
# -------------------------------------------------------------

IF(QT_LIBRARY)
MESSAGE("blabla")


SMP_QT_GENERATE_MOC(mainwindow.h)
SMP_QT_GENERATE_UI_HEADERS(mainwindow.ui)

# generate the UI headers
QT4_WRAP_UI(SMP_QT_UI_HEADERS ${SMP_QT_UI_FILES})
# Qt4 files that need to be preprocessed
QT4_WRAP_CPP(SMP_QT_MOC ${SMP_QT_FILES_FOR_MOC})

ENDIF(QT_LIBRARY)
# -------------------------------------------------------------
# --- Compiler and Compilation flags --------------------------
# -------------------------------------------------------------
SET(CMAKE_CXX_COMPILER g++)
SET(CMAKE_C_COMPILER g++)

IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  IF(APPLE)
    ADD_DEFINITIONS(-DMACOSX -x c++)
    SET (CMAKE_SHARED_LINKER_FLAGS "-dynamiclib -Wl,-headerpad_max_install_names,-undefined,dynamic_lookup")
  ELSE(APPLE)
    ADD_DEFINITIONS(-DLINUX)
  ENDIF(APPLE)
ELSE(UNIX)
 MESSAGE (FATAL_ERROR "Not supported Platform")
ENDIF(UNIX)

#Default mode is Debug
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_BUILD_TYPE}/bin/$ENV{HOSTTYPE}")
SET(LIBRARY_OUTPUT_PATH "${CMAKE_BUILD_TYPE}/lib/$ENV{HOSTTYPE}")

#Default Flags
#ADD_DEFINITIONS(-DKCD_MOVE3D -DPRINT_ERROR -DPRINT_INFO)

IF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  ADD_DEFINITIONS(-g -Wall -Wno-unknown-pragmas)
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  ADD_DEFINITIONS(-O2 -Wall -Wno-unknown-pragmas)
ENDIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
MESSAGE("QT source")
MESSAGE("${source_files}\n")
MESSAGE("QT moc")
MESSAGE("${SMP_QT_MOC}")
MESSAGE("QT ui")
MESSAGE("${SMP_QT_UI_HEADERS}")
ADD_EXECUTABLE(QSoftMotionPlanner ${source_files} ${SMP_QT_MOC} ${SMP_QT_UI_HEADERS})
TARGET_LINK_LIBRARIES(QSoftMotionPlanner softMotion ${LIBS} m)

ENDIF(QT_LIBRARY)
