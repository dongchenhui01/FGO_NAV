cmake_minimum_required(VERSION 3.16)
project(factor_graph_optimizer)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(underwater_nav_msgs REQUIRED)

# 系统依赖
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# 查找GTSAM
find_package(GTSAM REQUIRED)

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# 设置依赖列表
set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  sensor_msgs
  underwater_nav_msgs
)

# 核心库 (包含时间中心方法)
add_library(${PROJECT_NAME}_core
  src/underwater_fgo.cpp
  src/dvl_factor.cpp
  src/time_centric_methods.cpp
  src/lever_arm_compensation.cpp
)

target_link_libraries(${PROJECT_NAME}_core
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  gtsam
)

ament_target_dependencies(${PROJECT_NAME}_core ${dependencies})

# 传统水下导航节点
add_executable(underwater_navigation_node
  src/underwater_navigation_node.cpp
)

target_link_libraries(underwater_navigation_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(underwater_navigation_node ${dependencies})

# 时间中心导航节点 (基于gnssFGO论文)
add_executable(time_centric_navigation_node
  src/time_centric_navigation_node.cpp
)

target_link_libraries(time_centric_navigation_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(time_centric_navigation_node ${dependencies})

# 可视化节点
add_executable(visualization_node
  src/visualization_node.cpp
)

target_link_libraries(visualization_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(visualization_node ${dependencies})

# GPS参考轨迹发布器
add_executable(gps_reference_publisher
  src/gps_reference_publisher.cpp
)

target_link_libraries(gps_reference_publisher
  ${PROJECT_NAME}_core
)

ament_target_dependencies(gps_reference_publisher ${dependencies})

# 杆臂补偿测试程序
add_executable(test_lever_arm
  src/test_lever_arm.cpp
)

target_link_libraries(test_lever_arm
  ${PROJECT_NAME}_core
)

ament_target_dependencies(test_lever_arm ${dependencies})

# 安装
install(TARGETS
  ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  underwater_navigation_node
  time_centric_navigation_node
  visualization_node
  gps_reference_publisher
  test_lever_arm
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# 导出
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(${dependencies})

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Lint测试
  ament_lint_auto_find_test_dependencies()
  
  # 单元测试
  ament_add_gtest(test_underwater_fgo
    test/test_underwater_fgo.cpp
  )
  
  if(TARGET test_underwater_fgo)
    target_link_libraries(test_underwater_fgo ${PROJECT_NAME}_core)
    ament_target_dependencies(test_underwater_fgo ${dependencies})
  endif()
endif()

ament_package()
