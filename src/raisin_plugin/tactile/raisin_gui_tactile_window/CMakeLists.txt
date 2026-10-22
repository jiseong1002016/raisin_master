cmake_minimum_required(VERSION 3.15)
project(raisin_gui_tactile_window LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT COMMAND raisin_find_package)
  include(${CMAKE_CURRENT_SOURCE_DIR}/../../../../cmake/raisin_helper.cmake)
endif()
raisin_find_package(raisin_gui_base REQUIRED)
raisin_find_package(raisin_network REQUIRED)
raisin_find_package(raisin_tactile_sensor REQUIRED)

add_library(${PROJECT_NAME} SHARED raisin_gui_tactile_window.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PRIVATE raisin_gui_base raisin_network raisin_tactile_sensor)
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
if(BUILD_TESTING)
  add_executable(tactile_gui_manager_load_test test/gui_manager_load_test.cpp)
  set_target_properties(tactile_gui_manager_load_test PROPERTIES ENABLE_EXPORTS TRUE)
  get_filename_component(_raisin_gui_common_install_dir "${imgui_DIR}/../../.." ABSOLUTE)
  get_filename_component(_tactile_yaml_cpp_lib_dir "${yaml-cpp_DIR}/../.." ABSOLUTE)
  target_link_directories(tactile_gui_manager_load_test PRIVATE
    "${_tactile_yaml_cpp_lib_dir}" "${_raisin_gui_common_install_dir}/lib")
  target_link_libraries(tactile_gui_manager_load_test PRIVATE
    raisin_gui_base raisin_network raisin_tactile_sensor imgui
    "${_raisin_gui_common_install_dir}/lib/libglbinding.so" ${PROJECT_NAME})
  set(_tactile_gui_test_sdk_prefix "")
  foreach(_prefix IN LISTS CMAKE_PREFIX_PATH)
    if(IS_DIRECTORY "${_prefix}/resource" AND
       EXISTS "${_prefix}/config/raisin_network/config/params.yaml" AND
       EXISTS "${_prefix}/lib/libraisin_empty_encryption.so")
      set(_tactile_gui_test_sdk_prefix "${_prefix}")
      break()
    endif()
  endforeach()
  if(NOT _tactile_gui_test_sdk_prefix)
    message(FATAL_ERROR "The GUI manager test requires an SDK prefix with resources, network config, and empty encryption")
  endif()
  add_test(NAME tactile_gui_manager_load
    COMMAND tactile_gui_manager_load_test ${CMAKE_INSTALL_PREFIX} ${_tactile_gui_test_sdk_prefix})
  get_filename_component(_tactile_gui_stage_root "${CMAKE_INSTALL_PREFIX}/.." ABSOLUTE)
  set(_tactile_gui_test_library_dirs
    "${_tactile_gui_stage_root}/deps/yaml_compat"
    "${_raisin_gui_common_install_dir}/lib")
  foreach(_prefix IN LISTS CMAKE_PREFIX_PATH)
    if(IS_DIRECTORY "${_prefix}/lib" AND NOT "${_prefix}" STREQUAL "${CMAKE_INSTALL_PREFIX}")
      list(APPEND _tactile_gui_test_library_dirs "${_prefix}/lib")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES _tactile_gui_test_library_dirs)
  list(JOIN _tactile_gui_test_library_dirs ":" _tactile_gui_test_ld_library_path)
  set_tests_properties(tactile_gui_manager_load PROPERTIES
    TIMEOUT 15
    ENVIRONMENT "LD_LIBRARY_PATH=${_tactile_gui_test_ld_library_path}")
  set(_tactile_gui_build_library_hint
    "${CMAKE_CURRENT_BINARY_DIR}/src/raisin_gui_tactile_window/lib${PROJECT_NAME}.so")
  add_custom_command(TARGET tactile_gui_manager_load_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
      "${CMAKE_CURRENT_BINARY_DIR}/src/raisin_gui_tactile_window"
    COMMAND ${CMAKE_COMMAND} -E rm -f "${_tactile_gui_build_library_hint}"
    COMMAND ${CMAKE_COMMAND} -E create_symlink "$<TARGET_FILE:${PROJECT_NAME}>"
      "${_tactile_gui_build_library_hint}"
    COMMAND ${CMAKE_COMMAND} -E touch "${CMAKE_CURRENT_BINARY_DIR}/raisin.py"
    COMMAND ${CMAKE_COMMAND} -E rm -f "${CMAKE_CURRENT_BINARY_DIR}/install"
    COMMAND ${CMAKE_COMMAND} -E create_symlink "${CMAKE_INSTALL_PREFIX}"
      "${CMAKE_CURRENT_BINARY_DIR}/install")
endif()
