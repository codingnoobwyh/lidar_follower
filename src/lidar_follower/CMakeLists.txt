cmake_minimum_required(VERSION 3.8)
project(lidar_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(lidar_filter src/lidar_filter_node.cpp)
add_executable(lidar_follower src/lidar_follower_node.cpp)

ament_target_dependencies(lidar_filter 
    rclcpp
    sensor_msgs
    visualization_msgs
    )
ament_target_dependencies(lidar_follower 
    rclcpp
    sensor_msgs
    visualization_msgs
    )

install(TARGETS lidar_filter
    DESTINATION lib/${PROJECT_NAME})
install(TARGETS lidar_follower
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
    
install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz)
  

ament_package()
