cmake_minimum_required(VERSION 3.16)
project(data_preprocessing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(underwater_nav_msgs REQUIRED)

# 包含目录
include_directories(include)

# 设置依赖列表
set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  underwater_nav_msgs
)

# 核心库
add_library(${PROJECT_NAME}_core
  src/csv_reader.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core ${dependencies})

# CSV播放器节点
add_executable(csv_player_node
  src/csv_player_node.cpp
)

target_link_libraries(csv_player_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(csv_player_node ${dependencies})

# 安装
install(TARGETS
  ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  csv_player_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

# 导出
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(${dependencies})

ament_package()
