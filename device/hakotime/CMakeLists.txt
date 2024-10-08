cmake_minimum_required(VERSION 3.5.1)

add_compile_options(-fPIC)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")

project(HakoTimeDevice C CXX)

add_subdirectory(hakoniwa-core-cpp-client)

set(HAKO_CORE_SOURCE_DIR "${PROJECT_SOURCE_DIR}/hakoniwa-core-cpp-client")
set(HAKO_C_SOURCE_DIR "${HAKO_CORE_SOURCE_DIR}/src/hakoc")

add_compile_options(-g)
add_compile_options(-Wall)
add_compile_options(-Wunknown-pragmas)
add_compile_options(-Wtrigraphs)

set(CMAKE_C_FLAGS "-std=gnu99")
set(CMAKE_CXX_FLAGS "-std=c++17")

# 環境変数CORE_DIRが設定されているかをチェック
if(NOT DEFINED ENV{CORE_DIR})
    # 環境変数が設定されていない場合、CMake変数CORE_DIRにデフォルト値を設定
    set(CORE_DIR ../../../athrill/src)
else()
    # 環境変数が設定されている場合、その値をCMake変数CORE_DIRに設定
    set(CORE_DIR $ENV{CORE_DIR})
endif()

# 設定されたCORE_DIRを表示（デバッグ目的）
message(STATUS "CORE_DIR is set to: ${CORE_DIR}")

include_directories("${CORE_DIR}/inc")
include_directories("${CORE_DIR}/cpu")
include_directories("${CORE_DIR}/bus")
include_directories("${CORE_DIR}/device/mpu")
include_directories("${CORE_DIR}/device/peripheral/serial/fifo")
include_directories("${CORE_DIR}/lib")
include_directories(../lib/include)
include_directories(../lib/src)
include_directories(.)
include_directories("${HAKO_C_SOURCE_DIR}/src")

add_compile_definitions(OS_LINUX)
add_compile_definitions(ATHRILL_EXT_DEVICE)
add_compile_definitions(CXX_SAMPLE)

add_library(hakotime SHARED
    hakoniwa_device.c
    io/hakoniwa_device_io.c
    run/hakoniwa_device_run.c
)

target_compile_options(hakotime PRIVATE -fPIC)
target_link_libraries(hakotime
    hakoc
)
