cmake_minimum_required(VERSION 3.8)
project(unitree_a1_legged)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-Wno-missing-field-initializers)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_a1_legged_msgs REQUIRED)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()
set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

if("$ENV{UNITREE_LEGGED_SDK_PATH}" STREQUAL "")
    message(WARNING "the environment variable UNITREE_LEGGED_SDK_PATH seems to be empty, please execute 'source build_env_vars_export.sh' before compiling.")
endif()
message(STATUS "UNITREE_LEGGED_SDK_PATH is $ENV{UNITREE_LEGGED_SDK_PATH}")

include_directories(include
                    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
                    )

install(
        DIRECTORY ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/
        DESTINATION lib
        )

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/)

add_executable(ros2_udp src/ros2_udp.cpp)
target_link_libraries(ros2_udp ${EXTRA_LIBS})
ament_target_dependencies(ros2_udp rclcpp std_msgs unitree_a1_legged_msgs)

add_executable(ros2_udp_copy src/ros2_udp_copy.cpp)
target_link_libraries(ros2_udp_copy ${EXTRA_LIBS})
ament_target_dependencies(ros2_udp_copy rclcpp std_msgs unitree_a1_legged_msgs)


install(TARGETS
  ros2_udp
  ros2_udp_copy
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
