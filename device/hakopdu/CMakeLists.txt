cmake_minimum_required(VERSION 3.5.1)

add_compile_options(-fPIC)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")

project(hakopduDevice C CXX)

add_subdirectory(hakoniwa-core-cpp-client)

set(HAKO_CORE_SOURCE_DIR "${PROJECT_SOURCE_DIR}/hakoniwa-core-cpp-client")
set(HAKO_C_SOURCE_DIR "${HAKO_CORE_SOURCE_DIR}/src/hakoc")

set(HAKO_IO_PLUGIN "ev3")
add_compile_options(-g)
add_compile_options(-Wall)
add_compile_options(-Wunknown-pragmas)
add_compile_options(-Wtrigraphs)

set(CMAKE_C_FLAGS "-std=gnu99")
set(CMAKE_CXX_FLAGS "-std=c++17")

set(CORE_DIR ../../../athrill/src)

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

add_library(hakopdu SHARED
    hakoniwa_device.c
    io/plugins/${HAKO_IO_PLUGIN}/hakoniwa_device_io.c
    io/plugins/${HAKO_IO_PLUGIN}/packet/${HAKO_IO_PLUGIN}packet.c
    run/hakoniwa_device_run.c
)
target_include_directories(hakopdu
    PRIVATE io
    PRIVATE io/plugins/${HAKO_IO_PLUGIN}
    PRIVATE io/plugins/${HAKO_IO_PLUGIN}/packet
)

target_compile_options(hakopdu PRIVATE -fPIC)
target_link_libraries(hakopdu
    hakoc
)
