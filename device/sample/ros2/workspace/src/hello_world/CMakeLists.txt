cmake_minimum_required(VERSION 3.5)
project(hello_world)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(hello_world_msgs REQUIRED)

# ビルド設定の便利関数
function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp")
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

# talkerノードのビルド設定
custom_executable(talker)

# listenerノードのビルド設定
custom_executable(listener)

include_directories(include)

add_library(sample_client STATIC src/sample_client.cpp)
ament_target_dependencies(sample_client "rclcpp")

if(ENABLE_TEST)
    enable_testing()
    include(test/gtest.cmake)
    add_subdirectory(test)
endif()

ament_package()
