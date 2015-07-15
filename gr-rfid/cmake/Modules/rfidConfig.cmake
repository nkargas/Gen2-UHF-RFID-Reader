INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_RFID rfid)

FIND_PATH(
    RFID_INCLUDE_DIRS
    NAMES rfid/api.h
    HINTS $ENV{RFID_DIR}/include
        ${PC_RFID_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    RFID_LIBRARIES
    NAMES gnuradio-rfid
    HINTS $ENV{RFID_DIR}/lib
        ${PC_RFID_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(RFID DEFAULT_MSG RFID_LIBRARIES RFID_INCLUDE_DIRS)
MARK_AS_ADVANCED(RFID_LIBRARIES RFID_INCLUDE_DIRS)

