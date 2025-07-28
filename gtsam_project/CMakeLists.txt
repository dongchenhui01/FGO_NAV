cmake_minimum_required(VERSION 3.10)
project(AUV_Navigation)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 编译选项
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# 查找GTSAM
find_package(GTSAM REQUIRED)

# 查找Boost
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

# 查找Eigen3
find_package(Eigen3 REQUIRED)

# 查找TBB
find_package(TBB REQUIRED)

# 包含目录
include_directories(
    ${GTSAM_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    /usr/local/include
)

# 链接目录
link_directories(/usr/local/lib)

# 添加可执行文件
add_executable(auv_navigation src/auv_navigation.cpp)
add_executable(simple_navigation src/simple_navigation.cpp)

# 链接库
target_link_libraries(auv_navigation
    gtsam
    gtsam_unstable
    ${Boost_LIBRARIES}
    ${TBB_LIBRARIES}
    pthread
)

target_link_libraries(simple_navigation
    gtsam
    gtsam_unstable
    ${Boost_LIBRARIES}
    ${TBB_LIBRARIES}
    pthread
)

add_executable(basic_navigation src/basic_navigation.cpp)
target_link_libraries(basic_navigation
    gtsam
    gtsam_unstable
    ${Boost_LIBRARIES}
    ${TBB_LIBRARIES}
    pthread
)

# 设置输出目录
set_target_properties(auv_navigation PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
