#!/bin/bash

# 水下导航系统测试脚本
# 运行单元测试和集成测试

set -e

WORKSPACE_DIR="$HOME/underwater_nav_ws"
PROJECT_NAME="underwater_nav_fgo"

echo "=== 水下导航系统测试脚本 ==="
echo "工作空间: $WORKSPACE_DIR"
echo

# 检查工作空间是否存在
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "错误: 工作空间不存在: $WORKSPACE_DIR"
    echo "请先运行安装脚本创建工作空间"
    exit 1
fi

cd $WORKSPACE_DIR

# 检查是否已编译
if [ ! -d "install" ]; then
    echo "项目尚未编译，开始编译..."
    source /opt/ros/humble/setup.bash
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

# 设置环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "开始运行测试..."

# 1. 运行单元测试
echo
echo "=== 运行单元测试 ==="
colcon test --packages-select factor_graph_optimizer data_preprocessing state_estimator visualization

# 显示测试结果
echo
echo "=== 测试结果 ==="
colcon test-result --verbose

# 2. 生成示例数据
echo
echo "=== 生成测试数据 ==="
DATA_FILE="$WORKSPACE_DIR/test_data.csv"
python3 src/$PROJECT_NAME/scripts/generate_sample_data.py -o $DATA_FILE -d 60

if [ ! -f "$DATA_FILE" ]; then
    echo "错误: 无法生成测试数据"
    exit 1
fi

echo "测试数据已生成: $DATA_FILE"

# 3. 运行集成测试
echo
echo "=== 运行集成测试 ==="

# 启动导航系统（后台运行）
echo "启动导航系统..."
timeout 30s ros2 launch $PROJECT_NAME navigation.launch.py \
    data_file:=$DATA_FILE \
    use_rviz:=false &

LAUNCH_PID=$!

# 等待系统启动
sleep 5

# 检查节点是否运行
echo "检查系统状态..."
if ros2 node list | grep -q "underwater_navigation"; then
    echo "✓ 导航节点运行正常"
else
    echo "✗ 导航节点未运行"
    kill $LAUNCH_PID 2>/dev/null || true
    exit 1
fi

if ros2 node list | grep -q "csv_player"; then
    echo "✓ 数据播放节点运行正常"
else
    echo "✗ 数据播放节点未运行"
    kill $LAUNCH_PID 2>/dev/null || true
    exit 1
fi

# 检查话题发布
echo "检查话题发布..."
TOPICS=("/navigation/state" "/navigation/pose" "/navigation/trajectory")

for topic in "${TOPICS[@]}"; do
    if timeout 5s ros2 topic echo $topic --once > /dev/null 2>&1; then
        echo "✓ 话题 $topic 发布正常"
    else
        echo "✗ 话题 $topic 无数据"
    fi
done

# 测试服务调用
echo "测试服务调用..."
if timeout 5s ros2 service call /csv_player/play std_srvs/srv/Empty > /dev/null 2>&1; then
    echo "✓ 播放服务调用成功"
else
    echo "✗ 播放服务调用失败"
fi

# 等待一段时间让系统处理数据
echo "等待系统处理数据..."
sleep 10

# 检查导航状态
echo "检查导航状态..."
NAV_STATE=$(timeout 5s ros2 topic echo /navigation/state --once 2>/dev/null || echo "")

if echo "$NAV_STATE" | grep -q "is_initialized: true"; then
    echo "✓ 导航系统已初始化"
else
    echo "✗ 导航系统未初始化"
fi

if echo "$NAV_STATE" | grep -q "is_converged: true"; then
    echo "✓ 导航系统已收敛"
else
    echo "⚠ 导航系统未收敛（可能需要更多时间）"
fi

# 停止系统
echo "停止测试系统..."
kill $LAUNCH_PID 2>/dev/null || true
wait $LAUNCH_PID 2>/dev/null || true

# 4. 性能测试
echo
echo "=== 性能测试 ==="

# 生成更大的数据集
PERF_DATA_FILE="$WORKSPACE_DIR/perf_test_data.csv"
echo "生成性能测试数据..."
python3 src/$PROJECT_NAME/scripts/generate_sample_data.py -o $PERF_DATA_FILE -d 300

# 测量处理时间
echo "测试数据处理性能..."
START_TIME=$(date +%s)

timeout 60s ros2 launch $PROJECT_NAME navigation.launch.py \
    data_file:=$PERF_DATA_FILE \
    use_rviz:=false &

PERF_PID=$!

# 等待处理完成或超时
wait $PERF_PID 2>/dev/null || true

END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

echo "数据处理耗时: ${DURATION}秒"

if [ $DURATION -lt 60 ]; then
    echo "✓ 性能测试通过"
else
    echo "⚠ 性能测试超时"
fi

# 5. 清理
echo
echo "=== 清理测试文件 ==="
rm -f $DATA_FILE $PERF_DATA_FILE

echo
echo "=== 测试完成 ==="
echo "详细测试结果请查看: $WORKSPACE_DIR/log/latest_test/"
echo
echo "如果所有测试通过，系统已准备就绪！"
echo "使用方法:"
echo "1. 生成数据: python3 src/$PROJECT_NAME/scripts/generate_sample_data.py -o data.csv"
echo "2. 运行系统: ros2 launch $PROJECT_NAME navigation.launch.py data_file:=data.csv"
echo "3. 查看结果: ros2 topic echo /navigation/state"
