prefix=@CMAKE_INSTALL_PREFIX@
exec_prefix=@CMAKE_INSTALL_PREFIX@
libdir=@CMAKE_INSTALL_PREFIX@/lib
includedir=@CMAKE_INSTALL_PREFIX@/include
 
Name: softMotion-libs
Description: Trajectory generator with bounded jerk, acceleration and velocity
Version: @SOFTMOTION_VERSION@
Libs: -L${libdir} -lsoftMotion
Cflags: -I${includedir}
