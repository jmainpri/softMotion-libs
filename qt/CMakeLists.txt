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

file(GLOB source_files main.cpp QSoftMotionPlanner.cpp myPlot.cpp curve.cpp curve_nw.cpp viewer.cpp)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

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

  FIND_PACKAGE(QWT REQUIRED)
  INCLUDE_directories(${QWT_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${QWT_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${QWT_LIBRARIES})


#IF(ENABLE_DISPLAY)
  FIND_PACKAGE(QXML REQUIRED)
  INCLUDE_directories(${QXML_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${QXML_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${QXML_LIBRARIES})



  FIND_PACKAGE(QGLVIEWER REQUIRED)
  INCLUDE_directories(${QGLVIEWER_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${QGLVIEWER_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${QGLVIEWER_LIBRARIES})
#ENDIF(ENABLE_DISPLAY)

FIND_PACKAGE(LibXml2 REQUIRED)
INCLUDE_directories(${LIBXML2_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})
SET(LIBS ${LIBS} ${LIBXML2_LIBRARIES})

#-------------- GBM ---------------------------------------
#----------------------------------------------------------
FIND_PACKAGE(GBM REQUIRED)
INCLUDE_directories(${GBM_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GBM_INCLUDE_DIR})
SET(LIBS ${LIBS} ${GBM_LIBRARIES})

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

SMP_QT_GENERATE_MOC(QSoftMotionPlanner.h)
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
  ADD_DEFINITIONS(-g -Wall -Wextra -Wunused -Wno-unknown-pragmas)
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  ADD_DEFINITIONS(-O2 -Wall -Wunused -Wno-unknown-pragmas)
ENDIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")


ADD_EXECUTABLE(QSoftMotionPlanner ${source_files} ${SMP_QT_MOC} ${SMP_QT_UI_HEADERS})
SET_TARGET_PROPERTIES(QSoftMotionPlanner PROPERTIES LINKER_LANGUAGE CXX)
TARGET_LINK_LIBRARIES(QSoftMotionPlanner softMotion ${LIBS} m)

add_library(
softMotionPlanner
SHARED
${source_files}
)

SET_TARGET_PROPERTIES(softMotionPlanner PROPERTIES LINKER_LANGUAGE CXX)


# -------------------------------------------------------------
# --- Installation --------------------------------------------
# -------------------------------------------------------------

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "SoftMotionPlanner install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
INSTALL(TARGETS softMotionPlanner

        LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ CONFIGURATIONS ${CMAKE_BUILD_TYPE})

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "SoftMotionPlanner install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
INSTALL(TARGETS QSoftMotionPlanner
        RUNTIME DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE CONFIGURATIONS ${CMAKE_BUILD_TYPE})
 


FILE(GLOB HEADER "${CMAKE_SOURCE_DIR}/qt/*.h")
INSTALL(FILES ${HEADER} DESTINATION include/softMotion)


ENDIF(QT_LIBRARY)
