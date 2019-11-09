#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "realsense2::realsense-file" for configuration "Release"
set_property(TARGET realsense2::realsense-file APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(realsense2::realsense-file PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C;CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librealsense-file.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS realsense2::realsense-file )
list(APPEND _IMPORT_CHECK_FILES_FOR_realsense2::realsense-file "${_IMPORT_PREFIX}/lib/librealsense-file.a" )

# Import target "realsense2::realsense2" for configuration "Release"
set_property(TARGET realsense2::realsense2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(realsense2::realsense2 PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librealsense2.2.30.1.dylib"
  IMPORTED_SONAME_RELEASE "@rpath/librealsense2.2.30.dylib"
  )

list(APPEND _IMPORT_CHECK_TARGETS realsense2::realsense2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_realsense2::realsense2 "${_IMPORT_PREFIX}/lib/librealsense2.2.30.1.dylib" )

# Import target "realsense2::fw" for configuration "Release"
set_property(TARGET realsense2::fw APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(realsense2::fw PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libfw.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS realsense2::fw )
list(APPEND _IMPORT_CHECK_FILES_FOR_realsense2::fw "${_IMPORT_PREFIX}/lib/libfw.a" )

# Import target "realsense2::tm" for configuration "Release"
set_property(TARGET realsense2::tm APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(realsense2::tm PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libtm.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS realsense2::tm )
list(APPEND _IMPORT_CHECK_FILES_FOR_realsense2::tm "${_IMPORT_PREFIX}/lib/libtm.a" )

# Import target "realsense2::uvc_fw" for configuration "Release"
set_property(TARGET realsense2::uvc_fw APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(realsense2::uvc_fw PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libuvc_fw.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS realsense2::uvc_fw )
list(APPEND _IMPORT_CHECK_FILES_FOR_realsense2::uvc_fw "${_IMPORT_PREFIX}/lib/libuvc_fw.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
