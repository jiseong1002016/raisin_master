cmake_minimum_required(VERSION 3.15)
project(raisin_tactile_plugin LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT COMMAND raisin_find_package)
  include(${CMAKE_CURRENT_SOURCE_DIR}/../../../../cmake/raisin_helper.cmake)
endif()
raisin_find_package(raisin_plugin REQUIRED)
raisin_find_package(raisin_network REQUIRED)
raisin_find_package(Eigen3 REQUIRED)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR AND NOT TARGET raisin_tactile_sensor)
  add_subdirectory(../raisin_tactile_sensor ${CMAKE_CURRENT_BINARY_DIR}/sensor)
endif()
raisin_find_package(raisin_tactile_sensor REQUIRED)
add_library(raisin_tactile_plugin SHARED src/tactile_plugin.cpp src/backend.cpp src/spring_backend.cpp src/usb_protocol.cpp src/usb_backend.cpp)
target_include_directories(raisin_tactile_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(raisin_tactile_plugin PUBLIC raisin_plugin raisin_network
  raisin_tactile_sensor Eigen3::Eigen)
raisin_install(raisin_tactile_plugin raisin_plugin raisin_network raisin_tactile_sensor Eigen3)
install(DIRECTORY config/ DESTINATION config/raisin_tactile_plugin/config)
get_filename_component(_raisin_common_install_dir "${raisin_plugin_DIR}/../../.." ABSOLUTE)
get_filename_component(_tactile_stage_root "${CMAKE_INSTALL_PREFIX}/.." ABSOLUTE)
set(_tactile_test_library_dirs
  "${_tactile_stage_root}/deps/yaml_compat"
  "${_raisin_common_install_dir}/lib")
foreach(_prefix IN LISTS CMAKE_PREFIX_PATH)
  if(IS_DIRECTORY "${_prefix}/lib" AND NOT "${_prefix}" STREQUAL "${CMAKE_INSTALL_PREFIX}")
    list(APPEND _tactile_test_library_dirs "${_prefix}/lib")
  endif()
endforeach()
list(REMOVE_DUPLICATES _tactile_test_library_dirs)
list(JOIN _tactile_test_library_dirs ":" _tactile_test_ld_library_path)
add_executable(raisin_tactile_usb_process src/usb_process.cpp src/usb_backend.cpp src/usb_protocol.cpp)
target_include_directories(raisin_tactile_usb_process PRIVATE include)
target_link_libraries(raisin_tactile_usb_process PRIVATE raisin_network raisin_tactile_sensor)
install(TARGETS raisin_tactile_usb_process RUNTIME DESTINATION bin)
enable_testing()
add_executable(tactile_contract_test test/tactile_contract_test.cpp)
target_link_libraries(tactile_contract_test PRIVATE raisin_tactile_sensor raisin_network)
add_test(NAME tactile_contract COMMAND tactile_contract_test)
add_executable(tactile_raisin_test test/tactile_raisin_test.cpp)
target_link_libraries(tactile_raisin_test PRIVATE raisin_tactile_plugin)
add_test(NAME tactile_raisin COMMAND tactile_raisin_test ${CMAKE_CURRENT_SOURCE_DIR}/test/pad.urdf)
set_tests_properties(tactile_raisin PROPERTIES
  TIMEOUT 15
  ENVIRONMENT "LD_LIBRARY_PATH=${_tactile_test_ld_library_path}")
add_executable(tactile_plugin_lifecycle_test test/plugin_lifecycle_test.cpp)
target_link_libraries(tactile_plugin_lifecycle_test PRIVATE raisin_plugin raisin_network raisin_tactile_sensor ${CMAKE_DL_LIBS})
add_executable(tactile_usb_plugin_lifecycle_test test/usb_plugin_lifecycle_test.cpp src/usb_protocol.cpp)
target_include_directories(tactile_usb_plugin_lifecycle_test PRIVATE include)
set_target_properties(tactile_usb_plugin_lifecycle_test PROPERTIES ENABLE_EXPORTS TRUE)
target_link_libraries(tactile_usb_plugin_lifecycle_test PRIVATE raisin_plugin raisin_network raisin_tactile_sensor ${CMAKE_DL_LIBS})
add_dependencies(tactile_usb_plugin_lifecycle_test raisin_tactile_usb_process raisin_tactile_plugin)
set(_tactile_usb_process_hint "${CMAKE_CURRENT_BINARY_DIR}/src/raisin_tactile_plugin/raisin_tactile_usb_process")
add_custom_command(TARGET tactile_usb_plugin_lifecycle_test POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/src/raisin_tactile_plugin"
  COMMAND ${CMAKE_COMMAND} -E rm -f "${_tactile_usb_process_hint}"
  COMMAND ${CMAKE_COMMAND} -E create_symlink "$<TARGET_FILE:raisin_tactile_usb_process>" "${_tactile_usb_process_hint}"
  COMMAND ${CMAKE_COMMAND} -E touch "${CMAKE_CURRENT_BINARY_DIR}/raisin.py"
  COMMAND ${CMAKE_COMMAND} -E rm -f "${CMAKE_CURRENT_BINARY_DIR}/install"
  COMMAND ${CMAKE_COMMAND} -E create_symlink "${CMAKE_INSTALL_PREFIX}" "${CMAKE_CURRENT_BINARY_DIR}/install"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_INSTALL_PREFIX}/config"
  COMMAND ${CMAKE_COMMAND} -E rm -f "${CMAKE_INSTALL_PREFIX}/config/raisin_network"
  COMMAND ${CMAKE_COMMAND} -E create_symlink "${_raisin_common_install_dir}/config/raisin_network" "${CMAKE_INSTALL_PREFIX}/config/raisin_network"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_INSTALL_PREFIX}/lib"
  COMMAND ${CMAKE_COMMAND} -E rm -f "${CMAKE_INSTALL_PREFIX}/lib/libraisin_empty_encryption.so"
  COMMAND ${CMAKE_COMMAND} -E create_symlink "${_raisin_common_install_dir}/lib/libraisin_empty_encryption.so" "${CMAKE_INSTALL_PREFIX}/lib/libraisin_empty_encryption.so")
add_test(NAME tactile_usb_plugin_lifecycle
  COMMAND tactile_usb_plugin_lifecycle_test
    ${CMAKE_CURRENT_SOURCE_DIR}/test/pad.urdf
    $<TARGET_FILE:raisin_tactile_plugin>
    ${CMAKE_INSTALL_PREFIX}/../evidence/plugin_lifecycle_runs
    ${CMAKE_INSTALL_PREFIX})
set_tests_properties(tactile_usb_plugin_lifecycle PROPERTIES
  TIMEOUT 15
  ENVIRONMENT "LD_LIBRARY_PATH=${_tactile_test_ld_library_path}")
add_executable(tactile_usb_protocol_test test/usb_protocol_test.cpp src/usb_protocol.cpp)
target_include_directories(tactile_usb_protocol_test PRIVATE include)
target_compile_definitions(tactile_usb_protocol_test PRIVATE TACTILE_QT_FRAME_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/qt_frame_packets.bin")
add_test(NAME tactile_usb_protocol COMMAND tactile_usb_protocol_test)
add_executable(tactile_usb_backend_test test/usb_backend_test.cpp src/usb_backend.cpp src/usb_protocol.cpp)
target_include_directories(tactile_usb_backend_test PRIVATE include)
target_compile_definitions(tactile_usb_backend_test PRIVATE TACTILE_QT_FRAME_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/qt_frame_packets.bin")
add_test(NAME tactile_usb_backend COMMAND tactile_usb_backend_test)
set_tests_properties(tactile_usb_backend PROPERTIES TIMEOUT 15)
add_executable(tactile_v2_pipeline_test test/tactile_v2_pipeline_test.cpp)
target_include_directories(tactile_v2_pipeline_test PRIVATE include)
target_link_libraries(tactile_v2_pipeline_test PRIVATE raisin_tactile_sensor)
add_test(NAME tactile_v2_pipeline COMMAND tactile_v2_pipeline_test)
add_executable(tactile_shadow_test test/tactile_shadow_test.cpp)
target_link_libraries(tactile_shadow_test PRIVATE raisin_tactile_sensor)
add_test(NAME tactile_shadow COMMAND tactile_shadow_test)
add_executable(tactile_usb_process_integration_test test/usb_process_integration_test.cpp src/usb_protocol.cpp)
target_include_directories(tactile_usb_process_integration_test PRIVATE include)
target_compile_definitions(tactile_usb_process_integration_test PRIVATE
  TACTILE_QT_FRAME_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/qt_frame_packets.bin")
target_link_libraries(tactile_usb_process_integration_test PRIVATE raisin_network raisin_tactile_sensor)
add_dependencies(tactile_usb_process_integration_test raisin_tactile_usb_process)
add_test(NAME tactile_usb_process_integration
  COMMAND tactile_usb_process_integration_test $<TARGET_FILE:raisin_tactile_usb_process>)
set_tests_properties(tactile_usb_process_integration PROPERTIES
  TIMEOUT 15
  ENVIRONMENT "LD_LIBRARY_PATH=${_tactile_test_ld_library_path}")

add_test(NAME tactile_usb_dual_process_integration
  COMMAND tactile_usb_process_integration_test $<TARGET_FILE:raisin_tactile_usb_process> --dual)
set_tests_properties(tactile_usb_dual_process_integration PROPERTIES
  TIMEOUT 20 ENVIRONMENT "LD_LIBRARY_PATH=${_tactile_test_ld_library_path}")
