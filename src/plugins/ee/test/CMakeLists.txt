include(CTest)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(_ee_yaml_target yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(_ee_yaml_target yaml-cpp)
else()
  message(FATAL_ERROR "yaml-cpp package exported no supported CMake target")
endif()

if(BUILD_TESTING)
  set(_dongdrive_build_dir "${CMAKE_BINARY_DIR}")
  if(TARGET raisin_dongdrive)
    get_target_property(_dongdrive_build_dir raisin_dongdrive BINARY_DIR)
  endif()
  add_test(NAME ee_dongdrive_config_install
    COMMAND "${CMAKE_COMMAND}"
      "-DEE_BUILD_DIR=${_dongdrive_build_dir}"
      "-DDONGDRIVE_CONFIG_DIR=${RAISIN_DONGDRIVE_CONFIG_DIR}"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/dongdrive_config_install_test.cmake")

  add_executable(raipal_maintenance_node_check maintenance_node_check.cpp)
  target_include_directories(raipal_maintenance_node_check PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../raisin_raipal_ee/include")
  target_link_libraries(raipal_maintenance_node_check PRIVATE
    raisin_network raisin_parameter raisin_util raisin_thread_pool
    raisin_data_logger ${_ee_yaml_target})
  target_link_options(raipal_maintenance_node_check PRIVATE "LINKER:--disable-new-dtags")
  install(TARGETS raipal_maintenance_node_check RUNTIME DESTINATION bin)

  add_executable(ee_maintenance_sequence_test ee_maintenance_sequence_test.cpp)
  target_compile_options(ee_maintenance_sequence_test PRIVATE -UNDEBUG)
  target_include_directories(ee_maintenance_sequence_test PRIVATE
    "${RAISIN_MASTER_WORKSPACE}/src/raisin_raipal/src/raisin_raipal")
  add_test(NAME ee_maintenance_sequence COMMAND ee_maintenance_sequence_test)

  add_executable(ee_core_contract_test ee_core_contract_test.cpp)
  target_compile_options(ee_core_contract_test PRIVATE -UNDEBUG)
  target_include_directories(ee_core_contract_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../raisin_raipal_ee/include")
  target_link_libraries(ee_core_contract_test PRIVATE ${_ee_yaml_target})
  add_test(NAME ee_core_contract COMMAND ee_core_contract_test)

  add_executable(ee_core_pdo_test ee_core_pdo_test.cpp)
  target_compile_options(ee_core_pdo_test PRIVATE -UNDEBUG)
  target_compile_definitions(ee_core_pdo_test PRIVATE
    RAISIN_HARDWARE_PROFILE_FILE="${RAISIN_MASTER_WORKSPACE}/src/raisin_raipal/config/hardware_profiles.yaml")
  target_link_libraries(ee_core_pdo_test PRIVATE
    raisin_raipal_ee_drive_bridge ${_ee_yaml_target})
  add_test(NAME ee_core_pdo COMMAND ee_core_pdo_test)
  set_tests_properties(ee_core_pdo PROPERTIES TIMEOUT 15)

  if(EE_BUILD_INTEGRATION_TESTS)
    set(_ee_empty_encryption_library "")
    if(TARGET raisin_empty_encryption)
      set(_ee_empty_encryption_library "$<TARGET_FILE:raisin_empty_encryption>")
    endif()
    foreach(_sdk_prefix IN LISTS RAISIN_SDK_PREFIXES)
      if(EXISTS "${_sdk_prefix}/lib/libraisin_empty_encryption.so")
        set(_ee_empty_encryption_library
          "${_sdk_prefix}/lib/libraisin_empty_encryption.so")
        break()
      endif()
    endforeach()
    if(NOT _ee_empty_encryption_library)
      message(FATAL_ERROR
        "The integration fixture needs the supplied SDK's libraisin_empty_encryption.so")
    endif()

    set(_arm_urdf
      "${RAISIN_MASTER_WORKSPACE}/src/raisin_raipal/resource/urdf_raipal/urdf/raipal_stub-0_R.urdf")
    if(NOT EXISTS "${_arm_urdf}")
      message(FATAL_ERROR "The PluginManager integration fixture is missing: ${_arm_urdf}")
    endif()

    add_executable(ee_integration_test ee_integration_test.cpp)
  target_compile_options(ee_integration_test PRIVATE -UNDEBUG)
    target_compile_definitions(ee_integration_test PRIVATE
      RAISIN_TEST_ARM_URDF="${_arm_urdf}")
    target_link_libraries(ee_integration_test PRIVATE
      raisin_raipal_ee_drive_bridge raisin_plugin raisin_network raisin_parameter raisin_robot
      raisin_util raisim::raisim)
    set_target_properties(ee_integration_test PROPERTIES ENABLE_EXPORTS TRUE)
    add_dependencies(ee_integration_test raisin_raipal_ee_plugin)
    if(TARGET raisin_empty_encryption)
      add_dependencies(ee_integration_test raisin_empty_encryption)
    endif()
    add_test(NAME ee_integration
      COMMAND ee_integration_test "${CMAKE_INSTALL_PREFIX}"
        "${_ee_empty_encryption_library}")
    set_tests_properties(ee_integration PROPERTIES TIMEOUT 30)
  endif()
endif()

if(BUILD_TESTING)
  add_executable(ee_hardware_profile_test hardware_profile_test.cpp)
  target_compile_options(ee_hardware_profile_test PRIVATE -UNDEBUG)
  target_include_directories(ee_hardware_profile_test PRIVATE
    "${RAISIN_MASTER_WORKSPACE}/src/raisin_raipal/src/raisin_raipal"
    "${CMAKE_CURRENT_SOURCE_DIR}/../raisin_raipal_ee/include")
  target_compile_definitions(ee_hardware_profile_test PRIVATE
    RAISIN_HARDWARE_PROFILE_FILE="${RAISIN_MASTER_WORKSPACE}/src/raisin_raipal/config/hardware_profiles.yaml")
  target_link_libraries(ee_hardware_profile_test PRIVATE ${_ee_yaml_target})
  add_test(NAME ee_hardware_profile COMMAND ee_hardware_profile_test)
endif()
