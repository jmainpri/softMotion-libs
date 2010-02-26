
project(src)

file(GLOB source_files ./*)


#SET(CMAKE_C_COMPILER g++)
#SET(CMAKE_CXX_COMPILER g++)

SET(CMAKE_C_FLAGS "-lm ")

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})

add_library(
softMotion
SHARED
${source_files}
)

SET_TARGET_PROPERTIES(softMotion PROPERTIES LINKER_LANGUAGE C)

# -------------------------------------------------------------
# --- Installation --------------------------------------------
# -------------------------------------------------------------

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "SoftMotionPlanner install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
INSTALL(TARGETS softMotion

        LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ CONFIGURATIONS ${CMAKE_BUILD_TYPE})

FILE(GLOB HEADER "${CMAKE_SOURCE_DIR}/src/*.h")
INSTALL(FILES ${HEADER} DESTINATION include/softMotion)


