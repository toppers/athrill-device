cmake_minimum_required(VERSION 3.20)

project(athrill-hako-exdev VERSION 0.1.0 LANGUAGES C)

include(CTest)
include(GNUInstallDirs)

set(ATHRILL_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../athrill"
    CACHE PATH "Path to the Athrill common source tree")
get_filename_component(ATHRILL_SOURCE_DIR "${ATHRILL_SOURCE_DIR}" ABSOLUTE)
set(ATHRILL_SRC_DIR "${ATHRILL_SOURCE_DIR}/src")

option(HAKO_EXDEV_BUILD_HAKOTIME "Build the standalone hakotime EXDEV" ON)
option(HAKO_EXDEV_BUILD_HAKOPDU_EV3 "Build the EV3 PDU EXDEV" ON)

if(NOT EXISTS "${ATHRILL_SRC_DIR}/inc/athrill_exdev.h")
    message(FATAL_ERROR
        "Athrill EXDEV headers were not found under: ${ATHRILL_SOURCE_DIR}")
endif()
set(HAKO_EXDEV_ATHRILL_INCLUDE_DIRS
    "${ATHRILL_SRC_DIR}/inc"
    "${ATHRILL_SRC_DIR}/lib"
    "${ATHRILL_SRC_DIR}/cpu"
    "${ATHRILL_SRC_DIR}/device/mpu"
    "${ATHRILL_SRC_DIR}/device/peripheral/serial/fifo"
)

function(hako_exdev_configure_target target_name)
    target_compile_features("${target_name}" PRIVATE c_std_11)
    target_compile_definitions("${target_name}" PRIVATE ATHRILL_EXT_DEVICE)
    target_include_directories("${target_name}" PRIVATE
        ${HAKO_EXDEV_ATHRILL_INCLUDE_DIRS}
    )
    if(MSVC)
        target_compile_options("${target_name}" PRIVATE /utf-8 /W4)
        target_compile_definitions("${target_name}" PRIVATE
            _CRT_SECURE_NO_WARNINGS)
        target_include_directories("${target_name}" PRIVATE
            "${ATHRILL_SRC_DIR}/platform/windows/include")
    elseif(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options("${target_name}" PRIVATE -Wall -Wextra -Wpedantic)
        if(APPLE OR UNIX)
            target_compile_definitions("${target_name}" PRIVATE OS_LINUX)
        endif()
    endif()
endfunction()

function(hako_exdev_configure_install_rpath target_name)
    if(APPLE)
        set_target_properties("${target_name}" PROPERTIES
            INSTALL_RPATH "@loader_path")
    elseif(UNIX)
        set_target_properties("${target_name}" PROPERTIES
            INSTALL_RPATH "$ORIGIN")
    endif()
endfunction()

if(HAKO_EXDEV_BUILD_HAKOTIME OR HAKO_EXDEV_BUILD_HAKOPDU_EV3)
    add_library(hakotime_static STATIC
        src/hakotime/hakotime.c
    )
    add_library(hako-exdev::hakotime-static ALIAS hakotime_static)
    target_include_directories(hakotime_static PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
    set_target_properties(hakotime_static PROPERTIES
        OUTPUT_NAME hakotime
        POSITION_INDEPENDENT_CODE ON
    )
    hako_exdev_configure_target(hakotime_static)

    find_package(hakoniwa-core CONFIG REQUIRED)
    if(NOT TARGET hakoniwa-core::shakoc)
        message(FATAL_ERROR
            "hakoniwa-core package does not provide hakoniwa-core::shakoc")
    endif()

    add_library(hako_exdev_hakoniwa_runtime STATIC
        src/runtime/hakoniwa_runtime.c
    )
    target_include_directories(hako_exdev_hakoniwa_runtime PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(hako_exdev_hakoniwa_runtime PUBLIC
        hakoniwa-core::shakoc
    )
    set_target_properties(hako_exdev_hakoniwa_runtime PROPERTIES
        POSITION_INDEPENDENT_CODE ON
    )
    hako_exdev_configure_target(hako_exdev_hakoniwa_runtime)

    if(HAKO_EXDEV_BUILD_HAKOTIME)
        add_library(hakotime_shared SHARED
            src/hakotime/hakotime_exdev.c
        )
        add_library(hako-exdev::hakotime-shared ALIAS hakotime_shared)
        target_include_directories(hakotime_shared PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/include"
        )
        target_link_libraries(hakotime_shared PRIVATE
            hakotime_static
            hako_exdev_hakoniwa_runtime
        )
        set_target_properties(hakotime_shared PROPERTIES
            OUTPUT_NAME hakotime
            PREFIX "lib"
        )
        if(WIN32)
            set_target_properties(hakotime_shared PROPERTIES
                PREFIX ""
                ARCHIVE_OUTPUT_NAME hakotime_import
            )
            add_custom_command(TARGET hakotime_shared POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:hakoniwa-core::shakoc>
                    $<TARGET_FILE_DIR:hakotime_shared>
                VERBATIM
            )
        endif()
        hako_exdev_configure_target(hakotime_shared)
        hako_exdev_configure_install_rpath(hakotime_shared)

        install(TARGETS hakotime_shared
            ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
            LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
            RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        )
    endif()

    install(TARGETS hakotime_static
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    )
    install(FILES
        include/hako_exdev/hakotime.h
        include/hako_exdev/runtime.h
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/hako_exdev"
    )
endif()

if(HAKO_EXDEV_BUILD_HAKOPDU_EV3)
    add_library(hakopdu_ev3_core STATIC
        src/hakopdu_ev3/hakopdu_ev3.c
    )
    target_include_directories(hakopdu_ev3_core PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
    target_link_libraries(hakopdu_ev3_core PUBLIC hakotime_static)
    set_target_properties(hakopdu_ev3_core PROPERTIES
        POSITION_INDEPENDENT_CODE ON
    )
    hako_exdev_configure_target(hakopdu_ev3_core)

    add_library(hakopdu_ev3_shared SHARED
        src/hakopdu_ev3/hakopdu_ev3_exdev.c
    )
    add_library(hako-exdev::hakopdu-ev3 ALIAS hakopdu_ev3_shared)
    target_include_directories(hakopdu_ev3_shared PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )
    target_link_libraries(hakopdu_ev3_shared PRIVATE
        hakopdu_ev3_core
        hako_exdev_hakoniwa_runtime
    )
    set_target_properties(hakopdu_ev3_shared PROPERTIES
        OUTPUT_NAME hakopdu_ev3
        PREFIX "lib"
    )
    if(WIN32)
        set_target_properties(hakopdu_ev3_shared PROPERTIES
            PREFIX ""
            ARCHIVE_OUTPUT_NAME hakopdu_ev3_import
        )
        add_custom_command(TARGET hakopdu_ev3_shared POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:hakoniwa-core::shakoc>
                $<TARGET_FILE_DIR:hakopdu_ev3_shared>
            VERBATIM
        )
    endif()
    hako_exdev_configure_target(hakopdu_ev3_shared)
    hako_exdev_configure_install_rpath(hakopdu_ev3_shared)

    install(TARGETS hakopdu_ev3_shared
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    )
    install(FILES include/hako_exdev/hakopdu_ev3.h
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/hako_exdev"
    )
endif()

if(BUILD_TESTING AND HAKO_EXDEV_BUILD_HAKOTIME)
    add_executable(hakotime-core-test
        test/hakotime_test.c
    )
    target_include_directories(hakotime-core-test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )
    target_link_libraries(hakotime-core-test PRIVATE hakotime_static)
    hako_exdev_configure_target(hakotime-core-test)
    add_test(NAME hakotime-core COMMAND hakotime-core-test)

    add_executable(hakotime-exdev-load-test
        test/exdev_load_test.c
        "${ATHRILL_SRC_DIR}/lib/shared_library.c"
    )
    target_include_directories(hakotime-exdev-load-test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )
    target_link_libraries(hakotime-exdev-load-test PRIVATE
        ${CMAKE_DL_LIBS}
    )
    hako_exdev_configure_target(hakotime-exdev-load-test)
    add_test(NAME hakotime-exdev-load
        COMMAND hakotime-exdev-load-test $<TARGET_FILE:hakotime_shared>
    )
endif()

if(BUILD_TESTING AND HAKO_EXDEV_BUILD_HAKOPDU_EV3)
    add_executable(hakopdu-ev3-core-test
        test/hakopdu_ev3_test.c
    )
    target_include_directories(hakopdu-ev3-core-test PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )
    target_link_libraries(hakopdu-ev3-core-test PRIVATE hakopdu_ev3_core)
    hako_exdev_configure_target(hakopdu-ev3-core-test)
    add_test(NAME hakopdu-ev3-core COMMAND hakopdu-ev3-core-test)

    add_executable(hakopdu-ev3-exdev-load-test
        test/exdev_load_test.c
        "${ATHRILL_SRC_DIR}/lib/shared_library.c"
    )
    target_link_libraries(hakopdu-ev3-exdev-load-test PRIVATE
        ${CMAKE_DL_LIBS}
    )
    hako_exdev_configure_target(hakopdu-ev3-exdev-load-test)
    add_test(NAME hakopdu-ev3-exdev-load
        COMMAND hakopdu-ev3-exdev-load-test
            $<TARGET_FILE:hakopdu_ev3_shared>
    )
endif()
