cmake_minimum_required(VERSION 3.5.1)

add_compile_options(-fPIC)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")

project(HakoSampleDevice C CXX)

add_subdirectory(hakoniwa-core-cpp-client)

if(NOT DEFINED ENV{HAKO_CORE_SOURCE_DIR})
    set(HAKO_CORE_SOURCE_DIR "${PROJECT_SOURCE_DIR}/hakoniwa-core-cpp-client")
else()
    set(HAKO_CORE_SOURCE_DIR $ENV{HAKO_CORE_SOURCE_DIR})
endif()
set(HAKO_C_SOURCE_DIR "${HAKO_CORE_SOURCE_DIR}/src/hakoc")
if(NOT DEFINED ENV{CORE_DIR})
    set(CORE_DIR ../../../athrill/src)
else()
    set(CORE_DIR $ENV{CORE_DIR})
endif()
message(STATUS "HAKO_CORE_SOURCE_DIR is set to: ${HAKO_CORE_SOURCE_DIR}")
message(STATUS "CORE_DIR is set to: ${CORE_DIR}")

add_compile_options(-g)
add_compile_options(-Wall)
add_compile_options(-Wunknown-pragmas)
add_compile_options(-Wtrigraphs)

set(CMAKE_C_FLAGS "-std=gnu99")
set(CMAKE_CXX_FLAGS "-std=c++17")

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
include_directories("io")
include_directories("run")

add_compile_definitions(OS_LINUX)
add_compile_definitions(ATHRILL_EXT_DEVICE)
add_compile_definitions(CXX_SAMPLE)

add_library(hakosample SHARED
    hakoniwa_device.c
    io/hakoniwa_device_io.c
    run/hakoniwa_device_run.c
)

target_compile_options(hakosample PRIVATE -fPIC)
target_link_libraries(hakosample
    hakoc
)
