cmake_minimum_required(VERSION 3.15)
project(raipal_ee_staging LANGUAGES CXX)
include(CTest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(RAISIN_DEFAULT_MARCH "armv8.2-a+crypto+fp16+dotprod")
else()
  set(RAISIN_DEFAULT_MARCH "x86-64-v3")
endif()
set(RAISIN_MARCH "${RAISIN_DEFAULT_MARCH}" CACHE STRING "Target CPU architecture shared with the Raisin SDK")
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-march=${RAISIN_MARCH}" RAISIN_MARCH_SUPPORTED)
if(NOT RAISIN_MARCH_SUPPORTED)
  message(FATAL_ERROR "Unsupported RAISIN_MARCH='${RAISIN_MARCH}' for ${CMAKE_SYSTEM_PROCESSOR}")
endif()
add_compile_options("-march=${RAISIN_MARCH}")

set(EE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Staging build type (Debug or RelWithDebInfo)")
set_property(CACHE EE_BUILD_TYPE PROPERTY STRINGS Debug RelWithDebInfo)
if(NOT EE_BUILD_TYPE STREQUAL "Debug" AND NOT EE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  message(FATAL_ERROR "EE_BUILD_TYPE must be Debug or RelWithDebInfo")
endif()
set(CMAKE_BUILD_TYPE "${EE_BUILD_TYPE}" CACHE STRING "" FORCE)

foreach(_flag_var CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELWITHDEBINFO)
  if("${${_flag_var}}" MATCHES "(^|[ \t])-ffast-math([ \t]|$)|(^|[ \t])-Ofast([ \t]|$)")
    message(FATAL_ERROR "${_flag_var} contains a fast-math option; EE safety code must not use it")
  endif()
endforeach()

set(RAISIN_MASTER_WORKSPACE "" CACHE PATH "Raisin master workspace checkout")
set(RAISIN_SDK_PREFIXES "" CACHE STRING "Semicolon-separated read-only Raisin SDK prefixes")
set(RAISIN_DEPENDENCY_PREFIXES "" CACHE STRING "Semicolon-separated dependency prefixes")
set(RAISIN_EXTRA_INCLUDE_DIRS "" CACHE STRING "Semicolon-separated non-prefix include directories")
set(RAISIN_RUNTIME_LIBRARY_DIRS "" CACHE STRING "Semicolon-separated runtime/link library directories")
set(RAISIN_OPENCV_PREFIX "" CACHE PATH "Focused OpenCV install prefix used by the staged SDK build")
option(EE_BUILD_INTEGRATION_TESTS "Build the EE network/plugin/GUI integration test" ON)

if(NOT RAISIN_MASTER_WORKSPACE OR NOT IS_DIRECTORY "${RAISIN_MASTER_WORKSPACE}")
  message(FATAL_ERROR "Set RAISIN_MASTER_WORKSPACE to the workspace checkout containing src/raisin_raipal")
endif()
if(NOT RAISIN_SDK_PREFIXES)
  message(FATAL_ERROR "Set RAISIN_SDK_PREFIXES to one or more prepared Raisin SDK prefixes")
endif()
if(NOT CMAKE_INSTALL_PREFIX OR CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  message(FATAL_ERROR "Set CMAKE_INSTALL_PREFIX to a private staging install directory")
endif()

include("${RAISIN_MASTER_WORKSPACE}/cmake/raisin_helper.cmake")
list(PREPEND CMAKE_PREFIX_PATH ${RAISIN_SDK_PREFIXES} ${RAISIN_DEPENDENCY_PREFIXES})
set(_dongdrive_config_paths "${RAISIN_MASTER_WORKSPACE}/src/raisin_dongdrive/config")
foreach(_sdk_prefix IN LISTS RAISIN_SDK_PREFIXES)
  list(APPEND _dongdrive_config_paths "${_sdk_prefix}/config/raisin_dongdrive/config")
endforeach()
find_path(RAISIN_DONGDRIVE_CONFIG_DIR dongdrive_config/dongdrive_app.yaml
  PATHS ${_dongdrive_config_paths} NO_DEFAULT_PATH)
if(NOT RAISIN_DONGDRIVE_CONFIG_DIR)
  message(FATAL_ERROR "The real node requires the raisin_dongdrive runtime configuration")
endif()
install(DIRECTORY "${RAISIN_DONGDRIVE_CONFIG_DIR}/"
  DESTINATION config/raisin_dongdrive/config)
if(RAISIN_OPENCV_PREFIX)
  file(GLOB _opencv_config_candidates
    "${RAISIN_OPENCV_PREFIX}/lib/cmake/opencv4/OpenCVConfig.cmake"
    "${RAISIN_OPENCV_PREFIX}/lib/*/cmake/opencv4/OpenCVConfig.cmake"
    "${RAISIN_OPENCV_PREFIX}/share/OpenCV/OpenCVConfig.cmake")
  if(NOT _opencv_config_candidates)
    message(FATAL_ERROR
      "RAISIN_OPENCV_PREFIX has no OpenCVConfig.cmake: ${RAISIN_OPENCV_PREFIX}")
  endif()
  list(GET _opencv_config_candidates 0 _opencv_config)
  get_filename_component(_opencv_config_dir "${_opencv_config}" DIRECTORY)
  set(OpenCV_DIR "${_opencv_config_dir}" CACHE PATH
    "OpenCV config in the explicitly supplied staging SDK" FORCE)
  list(PREPEND CMAKE_PREFIX_PATH "${RAISIN_OPENCV_PREFIX}")
  find_package(OpenCV REQUIRED CONFIG PATHS "${OpenCV_DIR}" NO_DEFAULT_PATH)
  get_property(_opencv_imported_targets DIRECTORY PROPERTY IMPORTED_TARGETS)
  foreach(_opencv_target IN LISTS _opencv_imported_targets)
    if(NOT _opencv_target MATCHES "^opencv_" OR NOT TARGET ${_opencv_target})
      continue()
    endif()
    get_target_property(_opencv_configs ${_opencv_target} IMPORTED_CONFIGURATIONS)
    if(_opencv_configs MATCHES "(^|;)DEBUG(;|$)")
      set_property(TARGET ${_opencv_target} PROPERTY
        MAP_IMPORTED_CONFIG_RELWITHDEBINFO DEBUG)
    endif()
  endforeach()
  message(STATUS "Using staged OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")
endif()
# Prefer the explicitly supplied staging compatibility package over the
# workstation's normal raisin_gui_base package, whose config pulls optional
# FFmpeg development packages that this focused build does not need.
foreach(_dependency_prefix IN LISTS RAISIN_DEPENDENCY_PREFIXES)
  if(EXISTS "${_dependency_prefix}/raisin_gui_base/raisin_gui_baseConfig.cmake")
    set(raisin_gui_base_DIR "${_dependency_prefix}/raisin_gui_base" CACHE PATH
      "Staging compatibility config for raisin_gui_base" FORCE)
    break()
  endif()
endforeach()
if(RAISIN_EXTRA_INCLUDE_DIRS)
  include_directories(${RAISIN_EXTRA_INCLUDE_DIRS})
endif()
if(RAISIN_RUNTIME_LIBRARY_DIRS)
  link_directories(${RAISIN_RUNTIME_LIBRARY_DIRS})
endif()

# Discover the packaged sensor and GUI targets in this directory so the
# narrowly scoped imported-target path corrections below remain visible to all
# staged subdirectories.
raisin_find_package(raisin_gui_base REQUIRED)
raisin_find_package(raisin_sensor REQUIRED)

# These installed SDK targets carry a legacy OpenCV module list copied from a
# workstation with the full contrib set. The explicitly supplied focused SDK
# contains only the modules its shipped sensor/encoder binaries actually use.
# Keep every exported module target that OpenCVConfig supplied and drop only
# unavailable opencv_* metadata entries; without an explicit SDK prefix, leave
# the original link metadata untouched so missing libraries fail normally.
if(RAISIN_OPENCV_PREFIX)
  function(ee_remove_missing_opencv_link_metadata target)
    if(NOT TARGET ${target})
      return()
    endif()
    get_target_property(_interface_links ${target} INTERFACE_LINK_LIBRARIES)
    if(NOT _interface_links OR _interface_links MATCHES "-NOTFOUND$")
      return()
    endif()
    set(_filtered_links)
    foreach(_link_item IN LISTS _interface_links)
      if(_link_item MATCHES "^opencv_[A-Za-z0-9_]+$" AND NOT TARGET ${_link_item})
        message(STATUS
          "Dropping unavailable OpenCV metadata entry ${_link_item} from ${target}")
        continue()
      endif()
      list(APPEND _filtered_links "${_link_item}")
    endforeach()
    set_property(TARGET ${target} PROPERTY INTERFACE_LINK_LIBRARIES "${_filtered_links}")
  endfunction()

  ee_remove_missing_opencv_link_metadata(raisin_sensor)
  ee_remove_missing_opencv_link_metadata(raisin_av1_encoder)
endif()

function(ee_fix_legacy_imported_includes)
  get_property(_targets DIRECTORY PROPERTY IMPORTED_TARGETS)
  foreach(_target IN LISTS _targets)
    get_target_property(_includes ${_target} INTERFACE_INCLUDE_DIRECTORIES)
    if(NOT _includes OR _includes MATCHES "-NOTFOUND$")
      continue()
    endif()
    set(_fixed_includes ${_includes})
    foreach(_include IN LISTS _includes)
      if(_include MATCHES "^\\$<" OR NOT IS_ABSOLUTE "${_include}" OR IS_DIRECTORY "${_include}")
        continue()
      endif()
      string(TOLOWER "${_include}" _lowercase_include)
      if(IS_DIRECTORY "${_lowercase_include}")
        list(REMOVE_ITEM _fixed_includes "${_include}")
        list(APPEND _fixed_includes "${_lowercase_include}")
        message(STATUS "Corrected imported include path case: ${_include}")
      elseif(_include STREQUAL "/usr/include/libusb-1.0")
        list(REMOVE_ITEM _fixed_includes "${_include}")
        message(STATUS "Omitted missing optional realsense2::usb include path: ${_include}")
      else()
        message(FATAL_ERROR "Imported target ${_target} references missing include path: ${_include}")
      endif()
    endforeach()
    set_target_properties(${_target} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_fixed_includes}")
  endforeach()
endfunction()

ee_fix_legacy_imported_includes()

# One supplied SDK accidentally prefixed raisimRaipalConfig.cmake commands
# with semicolons. Load its otherwise valid imported target file directly,
# after resolving the two targets its interface references. The SDK stays
# read-only and the normal config path remains in place when it is valid.
raisin_find_package(raisim CONFIG REQUIRED)
raisin_find_package(Eigen3 REQUIRED)
foreach(_sdk_prefix IN LISTS RAISIN_SDK_PREFIXES)
  set(_raipal_package_dir "${_sdk_prefix}/lib/cmake/raisimRaipal")
  set(_raipal_package_config "${_raipal_package_dir}/raisimRaipalConfig.cmake")
  if(EXISTS "${_raipal_package_config}")
    file(STRINGS "${_raipal_package_config}" _malformed_commands REGEX "^;[ \\t]*(include|find_dependency)\\(")
    if(_malformed_commands)
      include("${_raipal_package_dir}/raisimRaipalTargets.cmake")
      message(STATUS "Loaded valid raisimRaipal targets around malformed SDK package config")
      break()
    endif()
  endif()
endforeach()

set(RAISIN_GENERATED_INCLUDE "")
foreach(_sdk_prefix IN LISTS RAISIN_SDK_PREFIXES)
  if(EXISTS "${_sdk_prefix}/include/raisin_tactile_sensor/msg/tactile_frame.hpp")
    set(RAISIN_GENERATED_INCLUDE "${_sdk_prefix}/include")
    break()
  endif()
endforeach()
if(NOT RAISIN_GENERATED_INCLUDE)
  message(FATAL_ERROR
    "No SDK prefix contains generated tactile_frame.hpp; pass the prepared tactile SDK prefix")
endif()

set(_raipal_source "${RAISIN_MASTER_WORKSPACE}/src/raisin_raipal")
set(_tactile_source "${_raipal_source}/tactile")
foreach(_path
    "${_raipal_source}/CMakeLists.txt"
    "${_tactile_source}/raisin_tactile_sensor/CMakeLists.txt"
    "${_tactile_source}/raisin_tactile_plugin/CMakeLists.txt"
    "${_tactile_source}/raisin_gui_tactile_window/CMakeLists.txt"
    "${CMAKE_CURRENT_SOURCE_DIR}/raisin_raipal_ee/CMakeLists.txt"
    "${CMAKE_CURRENT_SOURCE_DIR}/raisin_gui_raipal_ee_window/CMakeLists.txt")
  if(NOT EXISTS "${_path}")
    message(FATAL_ERROR "Required staged source is missing: ${_path}")
  endif()
endforeach()

# Build against the supplied SDK while taking tactile ABI headers and core/node
# sources from the selected workspace checkout.
add_subdirectory(
  "${_tactile_source}/raisin_tactile_sensor"
  "${CMAKE_CURRENT_BINARY_DIR}/tactile_sensor")
add_subdirectory(
  "${_raipal_source}"
  "${CMAKE_CURRENT_BINARY_DIR}/raisin_raipal")
foreach(_controller IN ITEMS joint_test ready)
  add_subdirectory(
    "${RAISIN_MASTER_WORKSPACE}/src/raipal_controller/raisin_raipal_${_controller}_controller"
    "${CMAKE_CURRENT_BINARY_DIR}/${_controller}_controller")
endforeach()
add_subdirectory(
  "${_tactile_source}/raisin_tactile_plugin"
  "${CMAKE_CURRENT_BINARY_DIR}/tactile_plugin")
add_subdirectory(
  "${_tactile_source}/raisin_gui_tactile_window"
  "${CMAKE_CURRENT_BINARY_DIR}/tactile_gui")
# The source package owns plugin targets and all normal install rules.

if(EE_BUILD_INTEGRATION_TESTS)
  add_subdirectory(test)
endif()

if(NOT CLANG_TIDY_EXE OR CLANG_TIDY_EXE MATCHES "-NOTFOUND$")
  message(STATUS
    "clang-tidy is unavailable; clearing only missing optional CXX_CLANG_TIDY target metadata")
  function(ee_clear_missing_clang_tidy directory)
    get_property(_targets DIRECTORY "${directory}" PROPERTY BUILDSYSTEM_TARGETS)
    foreach(_target IN LISTS _targets)
      get_target_property(_tidy ${_target} CXX_CLANG_TIDY)
      if(_tidy AND _tidy MATCHES "-NOTFOUND")
        set_property(TARGET ${_target} PROPERTY CXX_CLANG_TIDY "")
      endif()
    endforeach()
    get_property(_subdirectories DIRECTORY "${directory}" PROPERTY SUBDIRECTORIES)
    foreach(_subdirectory IN LISTS _subdirectories)
      ee_clear_missing_clang_tidy("${_subdirectory}")
    endforeach()
  endfunction()
  ee_clear_missing_clang_tidy("${CMAKE_CURRENT_SOURCE_DIR}")
endif()

function(ee_map_raisim_imports_to_release directory)
  get_property(_targets DIRECTORY "${directory}" PROPERTY IMPORTED_TARGETS)
  foreach(_target IN LISTS _targets)
    if(NOT _target MATCHES "^raisim(::|$)")
      continue()
    endif()
    get_target_property(_aliased_target ${_target} ALIASED_TARGET)
    if(_aliased_target)
      set(_property_target "${_aliased_target}")
    else()
      set(_property_target "${_target}")
    endif()
    get_target_property(_configs ${_property_target} IMPORTED_CONFIGURATIONS)
    if(_configs MATCHES "(^|;)RELEASE(;|$)")
      set_property(TARGET ${_property_target} PROPERTY MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
      set_property(TARGET ${_property_target} PROPERTY MAP_IMPORTED_CONFIG_DEBUG RELEASE)
    endif()
  endforeach()
  get_property(_subdirectories DIRECTORY "${directory}" PROPERTY SUBDIRECTORIES)
  foreach(_subdirectory IN LISTS _subdirectories)
    ee_map_raisim_imports_to_release("${_subdirectory}")
  endforeach()
endfunction()

ee_map_raisim_imports_to_release("${CMAKE_CURRENT_SOURCE_DIR}")

# These two paths come from this workstation's packaged SDK metadata. Correct
# only imported include entries whose case-folded directory exists, and omit
# the stale optional librealsense libusb header path when its dev headers are
# absent. No installed SDK file is changed.
function(ee_enable_test_assertions directory)
  get_property(_targets DIRECTORY "${directory}" PROPERTY BUILDSYSTEM_TARGETS)
  foreach(_target IN LISTS _targets)
    if(_target MATCHES "^ee_")
      get_target_property(_target_type ${_target} TYPE)
      if(_target_type STREQUAL "EXECUTABLE")
        target_compile_options(${_target} PRIVATE -UNDEBUG)
      endif()
    endif()
  endforeach()
  get_property(_subdirectories DIRECTORY "${directory}" PROPERTY SUBDIRECTORIES)
  foreach(_subdirectory IN LISTS _subdirectories)
    ee_enable_test_assertions("${_subdirectory}")
  endforeach()
endfunction()

ee_enable_test_assertions("${CMAKE_CURRENT_SOURCE_DIR}")
