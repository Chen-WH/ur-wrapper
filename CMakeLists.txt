cmake_minimum_required(VERSION 3.10)
project(ur-wrapper)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ur_client_library REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Executable Creation
add_executable(home src/home.cpp)
target_link_libraries(home ur_client_library::urcl)
ament_target_dependencies(home rclcpp ament_index_cpp)

#add_executable(p2p src/p2p.cpp)
#target_link_libraries(p2p ur_client_library::urcl)
#ament_target_dependencies(p2p rclcpp sensor_msgs ament_index_cpp)

#add_executable(point src/point.cpp)
#target_link_libraries(point ur_client_library::urcl)
#ament_target_dependencies(point rclcpp sensor_msgs trajectory_msgs ament_index_cpp)

add_executable(trajectory src/trajectory.cpp)
target_link_libraries(trajectory ur_client_library::urcl)
ament_target_dependencies(trajectory rclcpp sensor_msgs trajectory_msgs ament_index_cpp)

# Ensure resources are available in the build tree (so running from build/ finds them)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/resources DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Install resources into the package share so 'ros2 run' / installed package can find them
install(DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

# 安装可执行文件
install(TARGETS
    home
#    p2p
#    point
    trajectory
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()