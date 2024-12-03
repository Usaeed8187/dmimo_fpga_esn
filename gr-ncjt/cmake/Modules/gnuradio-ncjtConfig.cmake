find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_NCJT gnuradio-ncjt)

FIND_PATH(
    GR_NCJT_INCLUDE_DIRS
    NAMES gnuradio/ncjt/api.h
    HINTS $ENV{NCJT_DIR}/include
        ${PC_NCJT_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_NCJT_LIBRARIES
    NAMES gnuradio-ncjt
    HINTS $ENV{NCJT_DIR}/lib
        ${PC_NCJT_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-ncjtTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_NCJT DEFAULT_MSG GR_NCJT_LIBRARIES GR_NCJT_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_NCJT_LIBRARIES GR_NCJT_INCLUDE_DIRS)
