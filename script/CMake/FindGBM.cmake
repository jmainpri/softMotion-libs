# - Check for the presence of GBM
#
# The following variables are set when GBM is found:
#  HAVE_GBM       = Set to true, if all components of GBM
#                          have been found.
#  GBM_INCLUDE_DIR   = Include path for the header files of GBM
#  GBM_LIBRARIES  = Link these to use GBM

## -----------------------------------------------------------------------------
## Check for the header files

find_path (GBM_INCLUDE_DIR gbM/gb.h
  PATHS ${GBM_INC} /usr/local/include /usr/include /sw/include /opt/local/include  $ENV{ROBOTPKG_BASE}/include
  )
#if(${GBM_INCLUDE_DIR} MATCHES "GBM_INCLUDE_DIR-NOTFOUND")
#add_subdirectory(${BioMove3D_SOURCE_DIR}/other_libraries/gbM)
#endif(${GBM_INCLUDE_DIR} MATCHES "GBM_INCLUDE_DIR-NOTFOUND")
#find_path (GBM_INCLUDE_DIR gbM/gb.h  PATHS /usr/local/include /usr/include /sw/include /opt/local/include ${CMAKE_CURRENT_SOURCE_DIR}/other_libraries/gbM/build/install/include
#  )


## -----------------------------------------------------------------------------
## Check for the library

find_library (GBM_LIBRARIES gb
  PATHS ${GBM_LIB} /usr/local/lib /usr/lib /lib /sw/lib /opt/local/lib  $ENV{ROBOTPKG_BASE}/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (GBM_INCLUDE_DIR AND GBM_LIBRARIES)
  set (HAVE_GBM TRUE)
else (GBM_INCLUDE_DIR AND GBM_LIBRARIES)
  if (NOT GBM_FIND_QUIETLY)
    if (NOT GBM_INCLUDE_DIR)
      message (STATUS "Unable to find GBM header files!")
    endif (NOT GBM_INCLUDE_DIR)
    if (NOT GBM_LIBRARIES)
      message (STATUS "Unable to find GBM library files!")
    endif (NOT GBM_LIBRARIES)
  endif (NOT GBM_FIND_QUIETLY)
endif (GBM_INCLUDE_DIR AND GBM_LIBRARIES)

if (HAVE_GBM)
  if (NOT GBM_FIND_QUIETLY)
    message (STATUS "Found components for GBM")
    message (STATUS "GBM_INCLUDE_DIR = ${GBM_INCLUDE_DIR}")
    message (STATUS "GBM_LIBRARIES = ${GBM_LIBRARIES}")
  endif (NOT GBM_FIND_QUIETLY)
else (HAVE_GBM)
  if (GBM_FIND_REQUIRED)
    SET(GBM_LIB "" CACHE PATH "Paths where to additionally look for
    libs")
    SET(GBM_INC "" CACHE PATH "Paths where to additionally look for
    includes")
    message (FATAL_ERROR "Could not find GBM!")
  endif (GBM_FIND_REQUIRED)
endif (HAVE_GBM)

mark_as_advanced (
  HAVE_GBM
  GBM_LIBRARIES
  GBM_INCLUDE_DIR
  )
