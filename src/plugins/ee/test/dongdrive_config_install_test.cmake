set(_prefix "${EE_BUILD_DIR}/test/dongdrive_config_install")
file(REMOVE_RECURSE "${_prefix}")
execute_process(
  COMMAND "${CMAKE_COMMAND}" "-DCMAKE_INSTALL_PREFIX=${_prefix}"
    -DCMAKE_INSTALL_LOCAL_ONLY=1 -P "${EE_BUILD_DIR}/cmake_install.cmake"
  RESULT_VARIABLE _result)
if(NOT _result EQUAL 0)
  message(FATAL_ERROR "Dongdrive runtime configuration install failed")
endif()
file(GLOB_RECURSE _configs RELATIVE "${DONGDRIVE_CONFIG_DIR}"
  "${DONGDRIVE_CONFIG_DIR}/*.yaml")
if(NOT _configs)
  message(FATAL_ERROR "No Dongdrive runtime configurations found")
endif()
foreach(_config IN LISTS _configs)
  file(SHA256 "${DONGDRIVE_CONFIG_DIR}/${_config}" _source_sha)
  file(SHA256 "${_prefix}/config/raisin_dongdrive/config/${_config}" _installed_sha)
  if(NOT _source_sha STREQUAL _installed_sha)
    message(FATAL_ERROR "Installed Dongdrive configuration differs: ${_config}")
  endif()
endforeach()
