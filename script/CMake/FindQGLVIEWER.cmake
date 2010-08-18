# - Check for the presence of QGLVIEWER
#
# The following variables are set when QGLVIEWER is found:
#  HAVE_QGLVIEWER       = Set to true, if all components of QGLVIEWER
#                          have been found.
#  QGLVIEWER_INCLUDE_DIR   = Include path for the header files of QGLVIEWER
#  QGLVIEWER_LIBRARIES  = Link these to use QGLVIEWER

## -----------------------------------------------------------------------------
## Check for the header files

find_path (QGLVIEWER_INCLUDE_DIR qglviewer.h
  PATHS ${QGLVIEWER_INC} /usr/local/include /usr/include /sw/include /opt/local/include
  PATH_SUFFIXES QGLViewer
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (QGLVIEWER_LIBRARIES QGLViewer 
  PATHS ${QGLVIEWER_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib 
  )
if (NOT QGLVIEWER_LIBRARIES)
find_library (QGLVIEWER_LIBRARIES qglviewer-qt4 
  PATHS ${QGLVIEWER_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib 
  )
endif (NOT QGLVIEWER_LIBRARIES)

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARIES)
  set (HAVE_QGLVIEWER TRUE)
else (QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARIES)
  if (NOT QGLVIEWER_FIND_QUIETLY)
    if (NOT QGLVIEWER_INCLUDE_DIR)
      message (STATUS "Unable to find QGLVIEWER header files!")
    endif (NOT QGLVIEWER_INCLUDE_DIR)
    if (NOT QGLVIEWER_LIBRARIES)
      message (STATUS "Unable to find QGLVIEWER library files!")
    endif (NOT QGLVIEWER_LIBRARIES)
  endif (NOT QGLVIEWER_FIND_QUIETLY)
endif (QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARIES)

if (HAVE_QGLVIEWER)
  if (NOT QGLVIEWER_FIND_QUIETLY)
    message (STATUS "Found components for QGLVIEWER")
    message (STATUS "QGLVIEWER_INCLUDE_DIR = ${QGLVIEWER_INCLUDE_DIR}")
    message (STATUS "QGLVIEWER_LIBRARIES = ${QGLVIEWER_LIBRARIES}")
  endif (NOT QGLVIEWER_FIND_QUIETLY)
else (HAVE_QGLVIEWER)
  if (QGLVIEWER_FIND_REQUIRED)
    SET(QGLVIEWER_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(QGLVIEWER_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find QGLVIEWER!")
  endif (QGLVIEWER_FIND_REQUIRED)
endif (HAVE_QGLVIEWER)

mark_as_advanced (
  HAVE_QGLVIEWER
  QGLVIEWER_LIBRARIES
  QGLVIEWER_INCLUDE_DIR
  )
