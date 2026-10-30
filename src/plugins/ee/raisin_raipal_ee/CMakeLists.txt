cmake_minimum_required(VERSION 3.15)
project(raisin_raipal_ee_plugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT COMMAND raisin_find_package)
  include(${CMAKE_CURRENT_SOURCE_DIR}/../../../../cmake/raisin_helper.cmake)
endif()

raisin_find_package(raisin_plugin REQUIRED)
raisin_find_package(raisin_network REQUIRED)
raisin_find_package(Eigen3 REQUIRED)
raisin_find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/end_effector_plugin.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  raisin_plugin raisin_network Eigen3::Eigen yaml-cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

raisin_install(${PROJECT_NAME} raisin_plugin raisin_network Eigen3 yaml-cpp)
install(DIRECTORY config/ DESTINATION config/${PROJECT_NAME}/config)
install(DIRECTORY resource/ DESTINATION resource/raisin_raipal_ee/resource)

include(CTest)
if(BUILD_TESTING)
  add_executable(ee_raipal_ee_mechanics_test test/mechanics_test.cpp)
  target_include_directories(ee_raipal_ee_mechanics_test PRIVATE include)
  target_compile_options(ee_raipal_ee_mechanics_test PRIVATE -UNDEBUG)
  target_link_libraries(ee_raipal_ee_mechanics_test PRIVATE yaml-cpp)
  add_test(NAME ee_raipal_ee_mechanics COMMAND ee_raipal_ee_mechanics_test)
endif()
