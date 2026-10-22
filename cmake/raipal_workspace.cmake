if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "The RAIPAL workspace requires Linux")
endif()

set(RAISIM_SDK_ROOT "${CMAKE_SOURCE_DIR}/raisim2Lib" CACHE PATH "Default link to the versioned RaiSim SDK")
list(PREPEND CMAKE_PREFIX_PATH "${RAISIM_SDK_ROOT}/rayrai" "${RAISIM_SDK_ROOT}/raisim")
find_package(raisim 2.9.4 EXACT CONFIG REQUIRED
    PATHS "${RAISIM_SDK_ROOT}/raisim/lib/cmake/raisim" NO_DEFAULT_PATH)
find_package(rayrai CONFIG REQUIRED
    PATHS "${RAISIM_SDK_ROOT}/rayrai/lib/cmake/rayrai" NO_DEFAULT_PATH)
if(TARGET stb::stb AND NOT TARGET stb)
    add_library(stb ALIAS stb::stb)
endif()

set(BUILD_SHARED_LIBS ON)
set(RAISIM_RAIPAL_USE_RAISIM2 ON)
set(RAISIN_MASTER_WORKSPACE "${CMAKE_SOURCE_DIR}")
set(RAISIN_GENERATED_INCLUDE "${RAISIN_MASTER_INCLUDE}")
include_directories(BEFORE SYSTEM
    "${RAISIM_SDK_ROOT}/raisim/include"
    "${RAISIM_SDK_ROOT}/rayrai/include")
include_directories(BEFORE "${RAISIN_MASTER_INCLUDE}")
set(CMAKE_INSTALL_RPATH
    "${RAISIM_SDK_ROOT}/raisim/lib"
    "${RAISIM_SDK_ROOT}/rayrai/lib"
    "${RAISIM_SDK_ROOT}/raisim/lib/openusd"
    "$ORIGIN" "$ORIGIN/../lib" "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_BUILD_RPATH "${CMAKE_INSTALL_RPATH}")
add_link_options("LINKER:--disable-new-dtags")
include(CTest)

set(_raipal_sources
    raisin/raisin_shared_memory
    raisin/raisin_util
    raisin/raisin_parameter
    raisin/raisin_data_logger
    raisin/raisin_thread_pool
    raisin/raisin_encryption/raisin_encryption
    raisin/raisin_encryption/raisin_empty_encryption
    raisin/raisin_encryption/raisin_aes256_encryption
    raisin/raisin_network
    raisin/raisin_gui_interface
    raisin/raisin_tf
    raisin/raisin_robot
    raisin/raisin_joy_interface
    raisin/raisin_plugin
    raisin/raisin_password
    raisin/raisin_ffmpeg_image_transport
    raisin/raisin_joy
    raisin/raisin_audio
    raisin/raisin_av1_video_transport/raisin_av1_encoder
    raisin_ethercat
    raisin_robot_packages/raisin_actuator/raisin_raidrive
    raisin_dongdrive
    raisin_robot_packages/raisin_powerboard
    raisin_robot_packages/raisin_controller
    raisin_third_party_gui/imgui-node-editor
    raisin_gui/raisin_gui_base
    raisin_third_party_raipal
    raisin_raipal
    raipal_controller/raisin_raipal_controller_utils
    raipal_controller/raisin_raipal_rest_controller
    raipal_controller/raisin_raipal_ready_controller
    raipal_controller/raisin_raipal_joint_test_controller
    raipal_controller/raisin_raipal_joy_teleop_controller
    raipal_controller/raisin_raipal_trajectory_controller
    raipal_controller/raisin_raipal_motion_editor_controller
    raipal_controller/raisin_raipal_swing_controller
    raipal_controller/raisin_raipal_grasp_controller
    raisin_plugin)

foreach(_window IN ITEMS
    lock_profiler threadpool robot robot_dev raidrive robot_selection ffmpeg
    data_logger file_manager plot password profile_timeline raisim_tcp_viewer
    sensor_set_1 node_flow profile)
    list(APPEND _raipal_sources "raisin_gui/raisin_gui_basic_windows/raisin_gui_${_window}_window")
endforeach()
list(APPEND _raipal_sources raisin_gui/raisin_gui)

foreach(_source IN LISTS _raipal_sources)
    if(NOT EXISTS "${CMAKE_SOURCE_DIR}/src/${_source}/CMakeLists.txt")
        message(FATAL_ERROR "Missing RAIPAL source package: src/${_source}")
    endif()
    add_subdirectory("src/${_source}")
    get_filename_component(_package "${_source}" NAME)
    set(_assets config resource)
    if(_source STREQUAL "raisin_raipal" OR _source STREQUAL "raipal_controller/raisin_raipal_grasp_controller")
        set(_assets)
    endif()
    foreach(_asset IN LISTS _assets)
        if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/src/${_source}/${_asset}")
            install(DIRECTORY "${CMAKE_SOURCE_DIR}/src/${_source}/${_asset}/"
                DESTINATION "${_asset}/${_package}/${_asset}")
        endif()
    endforeach()
endforeach()

set(RAISIN_DONGDRIVE_CONFIG_DIR "${CMAKE_SOURCE_DIR}/src/raisin_dongdrive/config")
option(EE_BUILD_INTEGRATION_TESTS "Build the no-hardware EE plugin integration check" ON)
add_subdirectory(src/raisin_plugin/ee/test)
if(BUILD_TESTING)
    add_executable(raisin_installed_directory_test
        src/raisin/raisin_util/test/installed_directory_test.cpp)
    target_include_directories(raisin_installed_directory_test PRIVATE src/raisin/raisin_util/include)
    set_target_properties(raisin_installed_directory_test PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/test-installed/bin")
    add_test(NAME raisin_installed_directory COMMAND raisin_installed_directory_test)
endif()
