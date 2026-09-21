# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly.
cmake_minimum_required(VERSION 3.16)

set(DEBUG_VERSION "DebugMode")
set(RELEASE_VERSION "ReleaseMode")
set(COMPILE_MODE ${RELEASE_VERSION})            # Compile mode
string(TIMESTAMP COMPILE_TIME %y%m%d%H%M)        # Build timestamp
set(VERSION_MAJOR  1)    # Major version
set(VERSION_MINOR  0)    # Minor version
set(VERSION_LEVEL3 5)    # Patch version

# Debug mode: version + timestamp
if(${COMPILE_MODE} MATCHES ${DEBUG_VERSION})
    message(STATUS "Compile mode: ${COMPILE_MODE}")
    set(PROJECT_VER v${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_LEVEL3}-${COMPILE_TIME}-debug)
# Release mode: version only
else()
    message(STATUS "Compile mode: ${COMPILE_MODE}")
    set(PROJECT_VER v${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_LEVEL3})
endif()

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(PowerRune26-Armour)
