cmake_minimum_required(VERSION 3.15)
project(raipal_device_plugins LANGUAGES CXX)

include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/raisin_helper.cmake")
raisin_find_package(raisin_plugin REQUIRED)

if(NOT TARGET raisin_raipal_ee_plugin)
  add_subdirectory(ee/raisin_raipal_ee)
endif()
if(NOT TARGET raisin_gui_raipal_ee_window)
  add_subdirectory(ee/raisin_gui_raipal_ee_window)
endif()
add_subdirectory(raisin_d405_plugin)

if(NOT TARGET raisin_tactile_sensor)
  add_subdirectory(tactile/raisin_tactile_sensor)
endif()
add_subdirectory(tactile/raisin_tactile_plugin)
add_subdirectory(tactile/raisin_gui_tactile_window)
