#
# Copyright 2021-2025 Software Radio Systems Limited
#
# This file is part of srsRAN
#
# srsRAN is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as
# published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# srsRAN is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Affero General Public License for more details.
#
# A copy of the GNU Affero General Public License can be found in
# the LICENSE file in the top-level directory of this distribution
# and at http://www.gnu.org/licenses/.
#
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

macro(ADD_CXX_COMPILER_FLAG_IF_AVAILABLE flag have)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag(${flag} ${have})
    if (${have})
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${flag}")
    endif (${have})
endmacro(ADD_CXX_COMPILER_FLAG_IF_AVAILABLE)

string(REPLACE "-Wignored-qualifiers" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

########################################################################
# ENABLE_EXPORT
########################################################################

# ENABLE_EXPORT tells cmake to make some libaries available for other
# software to link to. If ON, the code must be compiled in PIC mode.
# We also add a dummy target that will depend on all the exported
# libraries to simplify their compilation.
if (ENABLE_EXPORT)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  add_custom_target(srsran_exported_libs)
endif (ENABLE_EXPORT)

# Simple macro that tags libraries to be exported and adds them to the
# dependencies of the dummy target.
macro(ADD_TO_EXPORTED_LIBS)
  if(ENABLE_EXPORT)
    # Tag libraries.
    install(TARGETS ${ARGV} EXPORT srsran_export)
    # Make libraries dependencies of the srsran_exported_libs dummy
    # target, which can be called to compile all exported libraries
    # at once.
    add_dependencies(srsran_exported_libs ${ARGV})
  endif(ENABLE_EXPORT)
endmacro(ADD_TO_EXPORTED_LIBS)

########################################################################
# Find dependencies
########################################################################

# Threads
find_package(Threads REQUIRED)

########################################################################
# Instruction Set Architecture setup
########################################################################
set(MTUNE "generic" CACHE STRING "Compiler -mtune flag. Default value is 'generic'")
if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(MARCH "armv8-a" CACHE STRING "Compiler -march flag. Default value is 'armv8-a' for aarch64.")
    message(STATUS "Detected aarch64 processor")
else (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(MARCH "native" CACHE STRING "Compiler -march flag. Default value is 'native' for x86-64.")
endif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")

add_cxx_compiler_flag_if_available("-march=${MARCH}" HAVE_MARCH)
if (NOT HAVE_MARCH)
    message(WARNING "The compiler does not support -march=${MARCH}, try setting a different value.")
endif (NOT HAVE_MARCH)

add_cxx_compiler_flag_if_available("-mtune=${MTUNE}" HAVE_MTUNE)
if (NOT HAVE_MTUNE)
    message(WARNING "The compiler does not support -mtune=${MTUNE}, try setting a different value.")
endif (NOT HAVE_MTUNE)

message(STATUS "ARCH value is ${MARCH}")
message(STATUS "TUNE value is ${MTUNE}")

# Append march and mtune to the compilation flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=${MARCH} -mtune=${MTUNE}")


########################################################################
# Compiler launcher setup
########################################################################

# Enable ccache if not already enabled
find_program(CCACHE_EXECUTABLE ccache)
mark_as_advanced(CCACHE_EXECUTABLE)
if (CCACHE_EXECUTABLE)
    foreach (LANG C CXX)
        if (NOT DEFINED CMAKE_${LANG}_COMPILER_LAUNCHER AND NOT CMAKE_${LANG}_COMPILER MATCHES ".*/ccache$")
            message(STATUS "Enabling ccache for ${LANG}")
            set(CMAKE_${LANG}_COMPILER_LAUNCHER ${CCACHE_EXECUTABLE} CACHE STRING "")
        endif ()
    endforeach ()
endif ()

include(CTest)
execute_process(COMMAND sed -i "s|MemoryCheckCommandOptions: |MemoryCheckCommandOptions: --verbose --trace-children=yes --time-stamp=yes --leak-check=full --show-leak-kinds=all --show-reachable=yes --exit-on-first-error=yes --error-exitcode=22 --suppressions=${CMAKE_SOURCE_DIR}/.memcheck-suppressions|" DartConfiguration.tcl
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

include_directories(include)
include_directories(external)

########################################################################
# Add the subdirectories
########################################################################

add_subdirectory(lib)




