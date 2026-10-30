cmake_minimum_required(VERSION 3.15)
project(raisin_gui_raipal_ee_window LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT COMMAND raisin_find_package)
  include(${CMAKE_CURRENT_SOURCE_DIR}/../../../../cmake/raisin_helper.cmake)
endif()
raisin_find_package(raisin_gui_base REQUIRED)
raisin_find_package(raisin_network REQUIRED)
raisin_find_package(yaml-cpp REQUIRED)
raisin_find_package(imgui REQUIRED)
# raisin_find_package is a function; find_package here as well so imgui_DIR is
# available in this directory scope for the staged manager-test link paths.
find_package(imgui REQUIRED)

add_library(${PROJECT_NAME} SHARED raisin_gui_raipal_ee_window.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../raisin_raipal_ee/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PRIVATE raisin_gui_base raisin_network yaml-cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION config/${PROJECT_NAME}/config)

include(CTest)
option(RAISIN_RAIPAL_EE_GUI_BUILD_TESTS "Build the isolated RAIPAL EE GUI manager test" OFF)
if(BUILD_TESTING AND RAISIN_RAIPAL_EE_GUI_BUILD_TESTS)
  set(_ee_empty_encryption_library "")
  foreach(_sdk_prefix IN LISTS RAISIN_SDK_PREFIXES)
    if(EXISTS "${_sdk_prefix}/lib/libraisin_empty_encryption.so")
      set(_ee_empty_encryption_library "${_sdk_prefix}/lib/libraisin_empty_encryption.so")
      break()
    endif()
  endforeach()
  if(NOT _ee_empty_encryption_library)
    message(FATAL_ERROR "The GUI manager test requires libraisin_empty_encryption.so in an SDK prefix")
  endif()

  add_executable(ee_raipal_ee_gui_manager_test test/gui_manager_test.cpp)
  set_target_properties(ee_raipal_ee_gui_manager_test PROPERTIES ENABLE_EXPORTS TRUE)
  target_include_directories(ee_raipal_ee_gui_manager_test PRIVATE
    ../raisin_raipal_ee/include)
  target_compile_options(ee_raipal_ee_gui_manager_test PRIVATE -UNDEBUG)
  get_filename_component(_raisin_gui_common_install_dir "${imgui_DIR}/../../.." ABSOLUTE)
  target_link_directories(ee_raipal_ee_gui_manager_test PRIVATE
    "${_raisin_gui_common_install_dir}/lib")
  target_link_libraries(ee_raipal_ee_gui_manager_test PRIVATE
    raisin_gui_base raisin_network imgui yaml-cpp
    "${_raisin_gui_common_install_dir}/lib/libglbinding.so" ${PROJECT_NAME})
  add_test(NAME ee_raipal_ee_gui_manager
    COMMAND ee_raipal_ee_gui_manager_test
      ${CMAKE_INSTALL_PREFIX} "${_ee_empty_encryption_library}"
      "${CMAKE_CURRENT_BINARY_DIR}/ee_gui_manager_test_artifacts")
  set_tests_properties(ee_raipal_ee_gui_manager PROPERTIES
    TIMEOUT 15
    ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${_raisin_gui_common_install_dir}/lib")
endif()
