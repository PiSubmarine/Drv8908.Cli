cmake_minimum_required (VERSION 3.25)
project("PiSubmarine.Drv8908")

set(_build_cmake_url "https://raw.githubusercontent.com/PiSubmarine/Build.CMake/main/PiSubmarine.Build.CMake.cmake")
set(_build_cmake_local "${CMAKE_BINARY_DIR}/PiSubmarine.Build.CMake.cmake")

if(NOT EXISTS "${_build_cmake_local}")
    message(STATUS "Downloading ${_build_cmake_url}...")
    file(DOWNLOAD "${_build_cmake_url}" "${_build_cmake_local}"
        INACTIVITY_TIMEOUT 60
        STATUS _status
        LOG _log
    )

    list(GET _status 0 _status_code)
    if(NOT _status_code EQUAL 0)
        message(FATAL_ERROR "Failed to download ${_build_cmake_url}: ${_log}")
    endif()
else()
    message(STATUS "Using cached file: ${_build_cmake_local}")
endif()

# Include the downloaded CMake file
include("${_build_cmake_local}")
PiSubmarineInitModule()