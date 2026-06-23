find_path(REALSENSE2_INCLUDE_DIR
    NAMES librealsense2/rs.hpp
    HINTS "C:/Program Files (x86)/Intel RealSense SDK 2.0/include"
)
find_library(REALSENSE2_LIBRARY
    NAMES realsense2
    HINTS "C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64"
)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(realsense2 DEFAULT_MSG
    REALSENSE2_LIBRARY REALSENSE2_INCLUDE_DIR)

if(realsense2_FOUND AND NOT TARGET realsense2::realsense2)
    add_library(realsense2::realsense2 UNKNOWN IMPORTED)
    set_target_properties(realsense2::realsense2 PROPERTIES
        IMPORTED_LOCATION "${REALSENSE2_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${REALSENSE2_INCLUDE_DIR}"
    )
endif()
