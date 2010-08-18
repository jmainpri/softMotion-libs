# - Check for the presence of QXML
#
# The following variables are set when QXML is found:
#  HAVE_QXML       = Set to true, if all components of QXML
#                          have been found.
#  QXML_INCLUDE_DIR   = Include path for the header files of QXML
#  QXML_LIBRARIES  = Link these to use QXML

## -----------------------------------------------------------------------------
## Check for the header files

find_path (QXML_INCLUDE_DIR qxml.h 
  PATHS ${QXML_INC} /usr/share/qt4/include /usr/local/include /usr/include /sw/include /opt/local/include
  PATH_SUFFIXES QtXml
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (QXML_LIBRARIES QtXml
  PATHS ${QXML_LIB}  /usr/share/qt4/lib /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (QXML_INCLUDE_DIR AND QXML_LIBRARIES)
  set (HAVE_QXML TRUE)
else (QXML_INCLUDE_DIR AND QXML_LIBRARIES)
  if (NOT QXML_FIND_QUIETLY)
    if (NOT QXML_INCLUDE_DIR)
      message (STATUS "Unable to find QXML header files!")
    endif (NOT QXML_INCLUDE_DIR)
    if (NOT QXML_LIBRARIES)
      message (STATUS "Unable to find QXML library files!")
    endif (NOT QXML_LIBRARIES)
  endif (NOT QXML_FIND_QUIETLY)
endif (QXML_INCLUDE_DIR AND QXML_LIBRARIES)

if (HAVE_QXML)
  if (NOT QXML_FIND_QUIETLY)
    message (STATUS "Found components for QXML")
    message (STATUS "QXML_INCLUDE_DIR = ${QXML_INCLUDE_DIR}")
    message (STATUS "QXML_LIBRARIES = ${QXML_LIBRARIES}")
  endif (NOT QXML_FIND_QUIETLY)
else (HAVE_QXML)
  if (QXML_FIND_REQUIRED)
    SET(QXML_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(QXML_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find QXML!")
  endif (QXML_FIND_REQUIRED)
endif (HAVE_QXML)

mark_as_advanced (
  HAVE_QXML
  QXML_LIBRARIES
  QXML_INCLUDE_DIR
  )
