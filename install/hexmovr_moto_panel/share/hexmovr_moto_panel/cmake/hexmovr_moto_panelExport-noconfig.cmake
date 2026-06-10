#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hexmovr_moto_panel::hexmovr_moto_panel" for configuration ""
set_property(TARGET hexmovr_moto_panel::hexmovr_moto_panel APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(hexmovr_moto_panel::hexmovr_moto_panel PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libhexmovr_moto_panel.so"
  IMPORTED_SONAME_NOCONFIG "libhexmovr_moto_panel.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hexmovr_moto_panel::hexmovr_moto_panel )
list(APPEND _IMPORT_CHECK_FILES_FOR_hexmovr_moto_panel::hexmovr_moto_panel "${_IMPORT_PREFIX}/lib/libhexmovr_moto_panel.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
