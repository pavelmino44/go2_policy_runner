# Copyright (c) 2026, unitree-go2-rl contributors.
# SPDX-License-Identifier: Apache-2.0
# Locate ONNX Runtime (C/C++ API, CPU).
#
# Search order:
#   1. -DONNXRUNTIME_ROOT=<dir>            (dir with include/ and lib/)
#   2. environment variable ONNXRUNTIME_ROOT
#   3. <package>/thirdparty/onnxruntime-linux-<x64|aarch64>-*   (scripts/fetch_onnxruntime.sh)
#   4. system paths (/usr/local, /usr)
#
# Defines the imported target onnxruntime::onnxruntime and ONNXRUNTIME_VENDORED (TRUE if the
# library must be installed together with this package).

set(ONNXRUNTIME_ROOT "" CACHE PATH "ONNX Runtime installation (contains include/ and lib/)")
if(NOT ONNXRUNTIME_ROOT AND DEFINED ENV{ONNXRUNTIME_ROOT})
  set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  set(_ort_arch "aarch64")
else()
  set(_ort_arch "x64")
endif()

set(ONNXRUNTIME_VENDORED FALSE)
if(NOT ONNXRUNTIME_ROOT)
  file(GLOB _ort_candidates LIST_DIRECTORIES true
    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/onnxruntime-linux-${_ort_arch}-*")
  if(_ort_candidates)
    list(SORT _ort_candidates)
    list(GET _ort_candidates -1 ONNXRUNTIME_ROOT)
    set(ONNXRUNTIME_VENDORED TRUE)
  endif()
endif()

find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS "${ONNXRUNTIME_ROOT}/include"
  PATH_SUFFIXES onnxruntime onnxruntime/core/session)
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime libonnxruntime.so.1
  HINTS "${ONNXRUNTIME_ROOT}/lib" "${ONNXRUNTIME_ROOT}/lib64")

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR
    "ONNX Runtime not found (arch: ${_ort_arch}).\n"
    "Run  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/fetch_onnxruntime.sh  or pass "
    "-DONNXRUNTIME_ROOT=/path/to/onnxruntime (e.g. unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0).")
endif()

message(STATUS "ONNX Runtime: ${ONNXRUNTIME_LIBRARY} (vendored: ${ONNXRUNTIME_VENDORED})")

if(NOT TARGET onnxruntime::onnxruntime)
  add_library(onnxruntime::onnxruntime SHARED IMPORTED)
  set_target_properties(onnxruntime::onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}")
endif()

# Directory to install next to our libraries when the runtime is vendored.
get_filename_component(ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
