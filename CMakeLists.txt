#    This file is part of openDarkEngine project
#    Copyright (C) 2005-2009 openDarkEngine team
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

cmake_minimum_required(VERSION 3.21)

project(OPDE LANGUAGES C CXX)

set(OPDE_VER_MAJOR "0")
set(OPDE_VER_MINOR "3")
set(OPDE_VER_PATCH "0")
set(REL_CODE_NAME  "El Motor Oscuro")

# for installation - directories with version in them (/usr/share/... etc.)
set(OPDE_SHORTNAME_WITH_VERSION "${CMAKE_PROJECT_NAME}-${OPDE_VER_MAJOR}.${OPDE_VER_MINOR}")
mark_as_advanced(OPDE_SHORTNAME_WITH_VERSION)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Build type selection
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# --- Options ---

option(GLOBAL_DEBUG "Build the debugging code" OFF)

option(GENERATE_NSIS "Creates Windows installer script" ON)

option(GENERATE_DOC "Generate documentation" OFF)

option(PROFILING
    "Build the sources with profiling support (Only Debug build type)"
    OFF)

option(FRAME_PROFILER
    "Build in support for frame profiling (will log frame profiling events)"
    OFF)

set(STATIC_LIBS OFF)
mark_as_advanced(STATIC_LIBS)

# Installation directories
set(OPDE_BINARY_INSTALL_DIR "bin")
mark_as_advanced(OPDE_BINARY_INSTALL_DIR)
set(OPDE_LIBRARY_INSTALL_DIR "lib")
mark_as_advanced(OPDE_LIBRARY_INSTALL_DIR)

set(OPDE_DATA_INSTALL_DIR share/${OPDE_SHORTNAME_WITH_VERSION})
set(OPDE_FULL_DATA_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${OPDE_DATA_INSTALL_DIR})
mark_as_advanced(OPDE_DATA_INSTALL_DIR)
mark_as_advanced(OPDE_FULL_DATA_INSTALL_DIR)

# Compiler profiling flags
if(PROFILING)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(PROFILER "-pg")
    endif()
else()
    set(PROFILER "")
endif()

# MinGW specific
if(MINGW)
    set(CMAKE_EXE_LINKER_FLAGS "-Wl,--enable-auto-image-base -Wl,--add-stdcall-alias -Wl,--enable-runtime-pseudo-reloc -Wl,--enable-auto-import")
    set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--enable-auto-image-base -Wl,--add-stdcall-alias -Wl,--enable-runtime-pseudo-reloc -Wl,--enable-auto-import")
endif()

set(CMAKE_CXX_WARNING_LEVEL 3)

# GCC/Clang flags
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_C_FLAGS_DEBUG "-g -O3 ${PROFILER}")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O3 ${PROFILER}")
    set(CMAKE_CXX_FLAGS_DISTRIBUTION "-O3 ${PROFILER}")
    set(CMAKE_C_FLAGS_DISTRIBUTION "-O3 ${PROFILER}")

    set(CMAKE_CXX_FLAGS "-O3 ${PROFILER}")
    set(CMAKE_C_FLAGS "-O3 ${PROFILER}")
endif()

# Various modules used for library path detections.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Global debug option
if(GLOBAL_DEBUG)
    message(STATUS "Debugging is ON")
    set(OPDE_DEBUG 1)
else()
    message(STATUS "Debugging is Off")
endif()

# --- Dependencies ---
set(ODE_DIR ${CMAKE_MODULE_PATH})

# Always-required dependencies
find_package(SDL2 REQUIRED)

if(SDL2_FOUND)
    message(STATUS "SDL2 Found OK")
    message(STATUS " - SDL2 includes ${SDL2_INCLUDE_DIR}")
    message(STATUS " - SDL2 libs ${SDL2_LIBRARIES}")
else()
    message(FATAL_ERROR "SDL2 not found!")
endif()

# ODE is used by physics service (headless-safe)
find_package(ODE REQUIRED)
if(ODE_FOUND)
    message(STATUS "ODE Found OK")
    message(STATUS " - ODE includes ${ODE_INCLUDE_DIR}")
    message(STATUS " - ODE libs ${ODE_LIBRARIES}")
else()
    message(FATAL_ERROR "ODE not found!")
endif()


# Documentation
if(GENERATE_DOC)
    find_package(Doxygen)
    find_program(TEXI2HTML texi2html)
    if(NOT DOT)
        find_program(DOT dot)
    endif()
    mark_as_advanced(TEXI2HTML DOT)

    include(FindPythonModule.cmake)
    find_python_module("epydoc" "EPYDOC")
endif()

set(DOC_DEPS_SATISFIED OFF)
mark_as_advanced(DOC_DEPS_SATISFIED)

# Some configuration checks to allow platform independence
include(ConfigureChecks.cmake)
configure_file(config.h.cmake ${CMAKE_BINARY_DIR}/config.h)

# Add the subdirectories which contain additional CMakeLists.txt files
add_subdirectory(src)
add_subdirectory(proto)
add_subdirectory(thirdparty)

# Additional fake_install dependencies
set(OPDE_DOC_INST_DEPS "")
mark_as_advanced(OPDE_DOC_INST_DEPS)

# Documentation generation
if(GENERATE_DOC AND DOC_DEPS_SATISFIED)
    set(OPDE_DOC_DIR share/${OPDE_SHORTNAME_WITH_VERSION})

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
                      ${CMAKE_BINARY_DIR}/Doxyfile)

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/writedoc.py.in
                      ${CMAKE_BINARY_DIR}/writedoc.py)

    if(DOXYGEN_EXECUTABLE)
        add_custom_target(doc ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile
            COMMENT "Building doxygen documentation" VERBATIM
        )
    else()
        message("Warning: Doxygen not found, docs target disabled.")
    endif()

    file(WRITE ${CMAKE_BINARY_DIR}/doc/python/html/temp.txt "temp")
    file(REMOVE ${CMAKE_BINARY_DIR}/doc/python/html/temp.txt)

    add_custom_target(pydoc ALL
        DEPENDS opdeScript
        COMMAND ${CMAKE_BINARY_DIR}/src/main/opdeScript ${CMAKE_BINARY_DIR}/writedoc.py
        COMMENT "Building Python API documentation" VERBATIM
    )

    mark_as_advanced(MANUAL_BUILDER)
    if(WIN32)
        set(MANUAL_BUILDER "${CMAKE_CURRENT_SOURCE_DIR}/doc/src/buildmanual.bat")
    else()
        set(MANUAL_BUILDER "/bin/sh" "${CMAKE_CURRENT_SOURCE_DIR}/doc/src/buildmanual.sh")
    endif()

    add_custom_target(manual ALL
        COMMAND ${MANUAL_BUILDER} ${CMAKE_CURRENT_SOURCE_DIR}/doc/src/ ${CMAKE_BINARY_DIR}/doc/manual/
        COMMENT "Building developer's manual" VERBATIM
    )

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/style.css
                      ${CMAKE_BINARY_DIR}/doc/style.css)

    install(DIRECTORY ${OPDE_BINARY_DIR}/doc DESTINATION ${OPDE_DOC_DIR})

    set(OPDE_DOC_INST_DEPS "doc" "pydoc" "manual")
endif()

if(GENERATE_NSIS)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/installer/opde-win.in
                      ${CMAKE_BINARY_DIR}/installer/opde-win.nsi @ONLY)
    message(STATUS "Installer script was generated in ${CMAKE_BINARY_DIR}/installer.")
endif()

# Optional testing local install (not automatic, use "make fake_install")
add_custom_target(fake_install
    "${CMAKE_COMMAND}"
    -D CMAKE_INSTALL_PREFIX:string=${CMAKE_CURRENT_BINARY_DIR}/fake_install/${CMAKE_INSTALL_PREFIX}
    -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_install.cmake"
)

message(STATUS "Your configuration seems to be OK!")
message(STATUS "")
message(STATUS "Your build parameters:")
message(STATUS " * Global debugging : ${GLOBAL_DEBUG}")
message(STATUS " * Build type       : ${CMAKE_BUILD_TYPE}")
message(STATUS " * Profilling       : ${PROFILING}")
message(STATUS " * Documentation    : ${GENERATE_DOC}")
message(STATUS " * C compiler       : ${CMAKE_C_COMPILER}")
message(STATUS " * C++ compiler     : ${CMAKE_CXX_COMPILER}")
message(STATUS " * Source directory : ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS " * Binary directory : ${CMAKE_CURRENT_BINARY_DIR}")
