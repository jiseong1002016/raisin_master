find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/../commands/workspace_raisim.py"
            "${CMAKE_INSTALL_PREFIX}"
    RESULT_VARIABLE _raipal_raisim_result
    OUTPUT_VARIABLE _raipal_raisim_root
    ERROR_VARIABLE _raipal_raisim_error
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT _raipal_raisim_result STREQUAL "0")
    message(FATAL_ERROR "${_raipal_raisim_error}")
endif()

set(raisim_DIR "${_raipal_raisim_root}/raisim/lib/cmake/raisim" CACHE PATH "Workspace RaiSim" FORCE)
set(rayrai_DIR "${_raipal_raisim_root}/rayrai/lib/cmake/rayrai" CACHE PATH "Workspace rayrai" FORCE)
set(CMAKE_PREFIX_PATH "${_raipal_raisim_root}/raisim;${_raipal_raisim_root}/rayrai;${_raipal_raisim_root}/thirdParty/Eigen3;${CMAKE_SOURCE_DIR}/install;${CMAKE_PREFIX_PATH}")
find_package(raisim CONFIG REQUIRED PATHS "${raisim_DIR}" NO_DEFAULT_PATH)
message(STATUS "RAIPAL workspace RaiSim SDK: ${_raipal_raisim_root}")
