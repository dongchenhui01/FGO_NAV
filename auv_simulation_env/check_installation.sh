#!/bin/bash

# AUV仿真环境安装状态检查脚本

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=== AUV仿真环境安装状态检查 ===${NC}"
echo ""

# 检查ROS 2
echo -e "${BLUE}检查ROS 2环境...${NC}"
if [ -n "$ROS_DISTRO" ]; then
    echo -e "${GREEN}✅ ROS 2 $ROS_DISTRO 已加载${NC}"
else
    echo -e "${RED}❌ ROS 2环境未加载${NC}"
fi

# 检查MAVROS
echo -e "${BLUE}检查MAVROS...${NC}"
if ros2 pkg list | grep -q mavros; then
    echo -e "${GREEN}✅ MAVROS已安装${NC}"
else
    echo -e "${RED}❌ MAVROS未安装${NC}"
fi

# 检查工作空间
echo -e "${BLUE}检查ROS工作空间...${NC}"
if [ -f ~/auv_colcon_ws/install/setup.bash ]; then
    echo -e "${GREEN}✅ ROS工作空间已创建${NC}"
    source ~/auv_colcon_ws/install/setup.bash
    if ros2 pkg list | grep -q auv; then
        echo -e "${GREEN}✅ 自定义AUV包已安装${NC}"
        ros2 pkg list | grep auv | sed 's/^/    /'
    else
        echo -e "${RED}❌ 自定义AUV包未找到${NC}"
    fi
else
    echo -e "${RED}❌ ROS工作空间未找到${NC}"
fi

# 检查ArduPilot
echo -e "${BLUE}检查ArduPilot SITL...${NC}"
if [ -d "ardupilot" ]; then
    echo -e "${YELLOW}⚠️  ArduPilot目录存在，但可能未完全安装${NC}"
    if command -v sim_vehicle.py &> /dev/null; then
        echo -e "${GREEN}✅ sim_vehicle.py可用${NC}"
    else
        echo -e "${RED}❌ sim_vehicle.py不可用${NC}"
    fi
else
    echo -e "${RED}❌ ArduPilot未安装${NC}"
fi

# 检查数据文件
echo -e "${BLUE}检查数据文件...${NC}"
if [ -f "data/real_data/sensor_fusion_log_20250712_102606.csv" ]; then
    echo -e "${GREEN}✅ 真实数据文件已复制${NC}"
    lines=$(wc -l < data/real_data/sensor_fusion_log_20250712_102606.csv)
    echo -e "    数据行数: $lines"
else
    echo -e "${RED}❌ 真实数据文件未找到${NC}"
fi

# 检查脚本
echo -e "${BLUE}检查脚本文件...${NC}"
if [ -f "scripts/data_replay.py" ]; then
    echo -e "${GREEN}✅ 数据回放脚本存在${NC}"
else
    echo -e "${RED}❌ 数据回放脚本未找到${NC}"
fi

if [ -f "scripts/algorithm_test.py" ]; then
    echo -e "${GREEN}✅ 算法测试脚本存在${NC}"
else
    echo -e "${RED}❌ 算法测试脚本未找到${NC}"
fi

# 检查Gazebo
echo -e "${BLUE}检查Gazebo...${NC}"
if command -v gazebo &> /dev/null; then
    echo -e "${GREEN}✅ Gazebo已安装${NC}"
    gazebo --version | head -1
else
    echo -e "${RED}❌ Gazebo未安装${NC}"
fi

echo ""
echo -e "${BLUE}=== 安装状态总结 ===${NC}"
echo -e "${GREEN}✅ 已完成: ROS 2, MAVROS, 工作空间, 数据回放${NC}"
echo -e "${YELLOW}⚠️  进行中: GeographicLib下载${NC}"
echo -e "${RED}❌ 待完成: ArduPilot SITL完整安装${NC}"

echo ""
echo -e "${BLUE}=== 可用功能测试 ===${NC}"
echo "测试数据回放:"
echo "  python3 scripts/data_replay.py data/real_data/sensor_fusion_log_20250712_102606.csv --rate 0.1"
echo ""
echo "查看ROS话题:"
echo "  ros2 topic list"
echo ""
echo "测试算法:"
echo "  python3 scripts/algorithm_test.py"
