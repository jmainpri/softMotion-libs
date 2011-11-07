# - Check for the presence of QWT
#
# The following variables are set when QWT is found:
#  HAVE_QWT       = Set to true, if all components of QWT
#                          have been found.
#  QWT_INCLUDE_DIR   = Include path for the header files of QWT
#  QWT_LIBRARIES  = Link these to use QWT

## -----------------------------------------------------------------------------
## Check for the header files

find_path (QWT_INCLUDE_DIR qwt_plot.h 
  PATHS ${QWT_INC} /usr/local/include /usr/include /sw/include /opt/local/include /usr/local/qwt-5.2.0/include
  PATH_SUFFIXES qwt qwt5
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (QWT_LIBRARIES qwt
  PATHS ${QWT_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib /usr/local/qwt-5.2.0/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (QWT_INCLUDE_DIR AND QWT_LIBRARIES)
  set (HAVE_QWT TRUE)
else (QWT_INCLUDE_DIR AND QWT_LIBRARIES)
  if (NOT QWT_FIND_QUIETLY)
    if (NOT QWT_INCLUDE_DIR)
      message (STATUS "Unable to find QWT header files!")
    endif (NOT QWT_INCLUDE_DIR)
    if (NOT QWT_LIBRARIES)
      message (STATUS "Unable to find QWT library files!")
    endif (NOT QWT_LIBRARIES)
  endif (NOT QWT_FIND_QUIETLY)
endif (QWT_INCLUDE_DIR AND QWT_LIBRARIES)

if (HAVE_QWT)
  if (NOT QWT_FIND_QUIETLY)
    message (STATUS "Found components for QWT")
    message (STATUS "QWT_INCLUDE_DIR = ${QWT_INCLUDE_DIR}")
    message (STATUS "QWT_LIBRARIES = ${QWT_LIBRARIES}")
  endif (NOT QWT_FIND_QUIETLY)
else (HAVE_QWT)
  if (QWT_FIND_REQUIRED)
    SET(QWT_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(QWT_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find QWT!")
  endif (QWT_FIND_REQUIRED)
endif (HAVE_QWT)

mark_as_advanced (
  HAVE_QWT
  QWT_LIBRARIES
  QWT_INCLUDE_DIR
  )
