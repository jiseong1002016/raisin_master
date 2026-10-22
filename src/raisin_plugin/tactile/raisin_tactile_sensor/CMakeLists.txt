cmake_minimum_required(VERSION 3.15)
project(raisin_tactile_sensor LANGUAGES CXX)
if(NOT COMMAND raisin_find_package)
  include(${CMAKE_CURRENT_SOURCE_DIR}/../../../../cmake/raisin_helper.cmake)
endif()
raisin_find_package(raisin_robot REQUIRED)
add_library(raisin_tactile_sensor INTERFACE)
target_compile_features(raisin_tactile_sensor INTERFACE cxx_std_20)
target_include_directories(raisin_tactile_sensor INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(DEFINED RAISIN_GENERATED_INCLUDE)
  set(_raisin_tactile_generated_include "${RAISIN_GENERATED_INCLUDE}")
else()
  get_filename_component(_raisin_workspace_root
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
  set(_raisin_tactile_generated_include "${_raisin_workspace_root}/generated/include")
endif()

foreach(_required_header
    raisin_tactile_sensor/msg/tactile_frame.hpp
    raisin_serialization_base.hpp
    std_msgs/msg/header.hpp
    builtin_interfaces/msg/time.hpp)
  if(NOT EXISTS "${_raisin_tactile_generated_include}/${_required_header}")
    message(FATAL_ERROR
      "Generated Raisin message header is missing: "
      "${_raisin_tactile_generated_include}/${_required_header}. "
      "Run Raisin message generation or set RAISIN_GENERATED_INCLUDE.")
  endif()
endforeach()

target_include_directories(raisin_tactile_sensor INTERFACE
  $<BUILD_INTERFACE:${_raisin_tactile_generated_include}>)
install(DIRECTORY "${_raisin_tactile_generated_include}/raisin_tactile_sensor/msg/"
    DESTINATION include/raisin_tactile_sensor/msg FILES_MATCHING PATTERN "*.hpp")
install(FILES
  "${_raisin_tactile_generated_include}/raisin_serialization_base.hpp"
  DESTINATION include)
install(DIRECTORY
  "${_raisin_tactile_generated_include}/std_msgs/msg/"
  DESTINATION include/std_msgs/msg FILES_MATCHING PATTERN "header.hpp")
install(DIRECTORY
  "${_raisin_tactile_generated_include}/builtin_interfaces/msg/"
  DESTINATION include/builtin_interfaces/msg FILES_MATCHING PATTERN "time.hpp")
target_link_libraries(raisin_tactile_sensor INTERFACE raisin_robot)
raisin_install(raisin_tactile_sensor raisin_robot)
