#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "coal::coal" for configuration "Release"
set_property(TARGET coal::coal APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(coal::coal PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/coal.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "assimp::assimp"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/coal.dll"
  )

list(APPEND _cmake_import_check_targets coal::coal )
list(APPEND _cmake_import_check_files_for_coal::coal "${_IMPORT_PREFIX}/lib/coal.lib" "${_IMPORT_PREFIX}/bin/coal.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
