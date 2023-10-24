cmake_minimum_required(VERSION 3.8)
project(unitree_a1_legged)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
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
link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)

install(
        DIRECTORY ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/
        DESTINATION lib
        )
        
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp unitree_a1_legged_msgs)

add_executable(ros2_udp src/ros2_udp.cpp)
target_link_libraries(ros2_udp ${EXTRA_LIBS})
ament_target_dependencies(ros2_udp rclcpp unitree_a1_legged_msgs)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ros2_udp
  talker
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
