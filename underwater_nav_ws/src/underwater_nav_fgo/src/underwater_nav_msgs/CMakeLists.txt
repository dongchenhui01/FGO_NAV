cmake_minimum_required(VERSION 3.16)
project(underwater_nav_msgs)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 消息文件
set(msg_files
  "msg/ImuData.msg"
  "msg/DvlData.msg"
  "msg/NavigationState.msg"
)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)

# 安装
ament_package()
