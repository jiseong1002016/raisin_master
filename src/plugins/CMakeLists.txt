cmake_minimum_required(VERSION 3.15)
project(raipal_device_plugins LANGUAGES CXX)
include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/raisin_helper.cmake")
raisin_find_package(raisin_plugin REQUIRED)
# The RAIPAL node owns the bundled EE targets; keep the camera plugin here.
add_subdirectory(raisin_d405_plugin)
