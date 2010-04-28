# - Check for the presence of GL
#
# The following variables are set when GL is found:
#  HAVE_GL       = Set to true, if all components of GL
#                          have been found.
#  GL_INCLUDE_DIR   = Include path for the header files of GL
#  GL_LIBRARIES  = Link these to use GL

## -----------------------------------------------------------------------------
## Check for the header files

find_path (GL_INCLUDE_DIR GL/gl.h
  PATHS ${GL_INC} /usr/local/include /usr/include /sw/include /opt/local/include
#  PATH_SUFFIXES /GL
  )

## -----------------------------------------------------------------------------
## Check for the library

find_library (GL_LIBRARIES GL
  PATHS ${GL_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (GL_INCLUDE_DIR AND GL_LIBRARIES)
  set (HAVE_GL TRUE)
else (GL_INCLUDE_DIR AND GL_LIBRARIES)
  if (NOT GL_FIND_QUIETLY)
    if (NOT GL_INCLUDE_DIR)
      message (STATUS "Unable to find GL header files!")
    endif (NOT GL_INCLUDE_DIR)
    if (NOT GL_LIBRARIES)
      message (STATUS "Unable to find GL library files!")
    endif (NOT GL_LIBRARIES)
  endif (NOT GL_FIND_QUIETLY)
endif (GL_INCLUDE_DIR AND GL_LIBRARIES)

if (HAVE_GL)
  if (NOT GL_FIND_QUIETLY)
    message (STATUS "Found components for GL")
    message (STATUS "GL_INCLUDE_DIR = ${GL_INCLUDE_DIR}")
    message (STATUS "GL_LIBRARIES = ${GL_LIBRARIES}")
  endif (NOT GL_FIND_QUIETLY)
else (HAVE_GL)
  if (GL_FIND_REQUIRED)
    message (FATAL_ERROR "Could not find GL!")
  endif (GL_FIND_REQUIRED)
endif (HAVE_GL)

mark_as_advanced (
  HAVE_GL
  GL_LIBRARIES
  GL_INCLUDE_DIR
  )
