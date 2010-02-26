project(tcl)




# -------------------------------------------------------------
# --- Installation --------------------------------------------
# -------------------------------------------------------------

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "SoftMotionPlanner install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)


FILE(GLOB TCL_FILES "${CMAKE_SOURCE_DIR}/tcl/*.tcl")
INSTALL(FILES ${TCL_FILES} DESTINATION share/softMotion)
