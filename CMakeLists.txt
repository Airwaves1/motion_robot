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

# 包含Eigen头文件
include_directories(${EIGEN3_INCLUDE_DIRS})

# 包含头文件目录
include_directories(include)


# 添加测试程序
add_executable(vrpn_topic_test 
  test/vrpn_topic_test.cpp
)

# 添加新的测试节点
add_executable(vrpn_test_node 
  src/vrpn_test_node.cpp
  src/joint_mapper.cpp
  src/topic_data_getter.cpp
)

# 设置依赖
# ament_target_dependencies(vrpn_g1_controller 
#   rclcpp 
#   std_msgs 
#   geometry_msgs 
#   unitree_hg
#   sensor_msgs
#   eigen3
# )

ament_target_dependencies(vrpn_topic_test 
  rclcpp 
  std_msgs 
  geometry_msgs
  sensor_msgs
)

ament_target_dependencies(vrpn_test_node 
  rclcpp 
  std_msgs 
  geometry_msgs
  sensor_msgs
  eigen3_cmake_module
)

# 链接Eigen库
target_link_libraries(vrpn_test_node ${EIGEN3_LIBRARIES})


# 安装可执行文件
# install(TARGETS 
#   vrpn_g1_controller
#   vrpn_topic_test
#   DESTINATION bin/${PROJECT_NAME}
# )

install(TARGETS vrpn_topic_test vrpn_test_node)

# install(TARGETS vrpn_g1_controller
#   DESTINATION bin/vrpn_g1_controller
# )

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装launch文件（如果存在）
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# 安装配置文件（如果存在）
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
