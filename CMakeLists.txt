cmake_minimum_required(VERSION 3.16)
project(motion_robot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置构建类型为Release以获得更好的性能
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(unitree_hg REQUIRED)

# 包含Eigen头文件
include_directories(${EIGEN3_INCLUDE_DIRS})

# 包含头文件目录
include_directories(include)


# 添加主程序
add_executable(motion_robot_node 
  src/main.cpp
  src/motion_robot.cpp
  src/g1_controller.cpp
  src/motor_crc.cpp
  src/topic_data_getter.cpp
)

ament_target_dependencies(motion_robot_node 
  rclcpp 
  std_msgs 
  geometry_msgs
  sensor_msgs
  unitree_hg
  Eigen3
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)
 
install(TARGETS motion_robot_node)


# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
