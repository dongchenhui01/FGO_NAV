#!/bin/bash

# 完整的因子图方法对比测试脚本
# 包含数据检查、方法对比、可视化分析的完整流程

set -e

# 配置参数
DATA_FILE="/home/dongchenhui/underwater_nav_fgo/data/sensor_fusion_log_20250712_102606.csv"
OUTPUT_DIR="/tmp/fgo_comparison_$(date +%Y%m%d_%H%M%S)"
WORKSPACE_DIR="$HOME/underwater_nav_ws"

echo "=== 因子图方法完整对比测试 ==="
echo "数据文件: $DATA_FILE"
echo "输出目录: $OUTPUT_DIR"
echo "工作空间: $WORKSPACE_DIR"
echo

# 检查必要文件
if [ ! -f "$DATA_FILE" ]; then
    echo "错误: 数据文件不存在: $DATA_FILE"
    exit 1
fi

if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "错误: ROS2工作空间不存在: $WORKSPACE_DIR"
    echo "请先编译项目"
    exit 1
fi

# 创建输出目录
mkdir -p "$OUTPUT_DIR"

# 设置ROS2环境
echo "设置ROS2环境..."
source /opt/ros/galactic/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

# 步骤1: 数据格式检查
echo
echo "=== 步骤1: 数据格式检查 ==="
echo "数据文件基本信息:"
echo "  文件大小: $(du -h $DATA_FILE | cut -f1)"
echo "  数据行数: $(wc -l < $DATA_FILE)"
echo "  数据列数: $(head -1 $DATA_FILE | tr ',' '\n' | wc -l)"

echo
echo "数据格式预览:"
head -3 "$DATA_FILE"

echo
echo "数据时间范围:"
echo "  开始时间: $(head -2 $DATA_FILE | tail -1 | cut -d',' -f1)"
echo "  结束时间: $(tail -1 $DATA_FILE | cut -d',' -f1)"

# 检查GPS数据质量
GPS_VALID_COUNT=$(awk -F',' 'NR>1 && $19>=3 && $25!=0 && $26!=0 {count++} END {print count+0}' "$DATA_FILE")
echo "  GPS有效点数: $GPS_VALID_COUNT"

# 步骤2: 运行传统因子图方法
echo
echo "=== 步骤2: 运行传统因子图方法 ==="

TRAD_OUTPUT="$OUTPUT_DIR/traditional_results"
mkdir -p "$TRAD_OUTPUT"

echo "启动传统导航系统..."
timeout 300s ros2 launch underwater_nav_fgo navigation.launch.py \
    data_file:="$DATA_FILE" \
    use_rviz:=false \
    output_directory:="$TRAD_OUTPUT" \
    > "$TRAD_OUTPUT/traditional_log.txt" 2>&1 &

TRAD_PID=$!
echo "传统方法进程ID: $TRAD_PID"

# 等待传统方法完成
echo "等待传统方法处理完成..."
wait $TRAD_PID
TRAD_EXIT_CODE=$?

if [ $TRAD_EXIT_CODE -eq 0 ]; then
    echo "✓ 传统方法执行成功"
else
    echo "✗ 传统方法执行失败 (退出码: $TRAD_EXIT_CODE)"
    echo "查看日志: $TRAD_OUTPUT/traditional_log.txt"
fi

# 步骤3: 运行时间中心因子图方法
echo
echo "=== 步骤3: 运行时间中心因子图方法 ==="

TC_OUTPUT="$OUTPUT_DIR/time_centric_results"
mkdir -p "$TC_OUTPUT"

echo "启动时间中心导航系统..."
timeout 300s ros2 launch underwater_nav_fgo time_centric_navigation.launch.py \
    data_file:="$DATA_FILE" \
    use_rviz:=false \
    time_window_size:=2.0 \
    interpolation_method:=gp \
    enable_continuous_query:=true \
    output_directory:="$TC_OUTPUT" \
    > "$TC_OUTPUT/time_centric_log.txt" 2>&1 &

TC_PID=$!
echo "时间中心方法进程ID: $TC_PID"

# 等待时间中心方法完成
echo "等待时间中心方法处理完成..."
wait $TC_PID
TC_EXIT_CODE=$?

if [ $TC_EXIT_CODE -eq 0 ]; then
    echo "✓ 时间中心方法执行成功"
else
    echo "✗ 时间中心方法执行失败 (退出码: $TC_EXIT_CODE)"
    echo "查看日志: $TC_OUTPUT/time_centric_log.txt"
fi

# 步骤4: 生成对比分析
echo
echo "=== 步骤4: 生成对比分析 ==="

# 安装Python依赖
echo "检查Python依赖..."
python3 -c "import matplotlib, seaborn, pandas, numpy" 2>/dev/null || {
    echo "安装Python可视化依赖..."
    pip3 install --user matplotlib seaborn pandas numpy
}

# 运行对比分析
echo "运行对比分析脚本..."
python3 /home/dongchenhui/underwater_nav_fgo/scripts/compare_methods.py \
    --data_file "$DATA_FILE" \
    --output_dir "$OUTPUT_DIR" \
    > "$OUTPUT_DIR/comparison_log.txt" 2>&1

if [ $? -eq 0 ]; then
    echo "✓ 对比分析完成"
else
    echo "✗ 对比分析失败"
    echo "查看日志: $OUTPUT_DIR/comparison_log.txt"
fi

# 步骤5: 生成可视化图表
echo
echo "=== 步骤5: 生成可视化图表 ==="

echo "生成可视化图表..."
python3 /home/dongchenhui/underwater_nav_fgo/scripts/visualize_comparison.py \
    --data_file "$DATA_FILE" \
    --results_dir "$OUTPUT_DIR" \
    > "$OUTPUT_DIR/visualization_log.txt" 2>&1

if [ $? -eq 0 ]; then
    echo "✓ 可视化图表生成完成"
else
    echo "✗ 可视化图表生成失败"
    echo "查看日志: $OUTPUT_DIR/visualization_log.txt"
fi

# 步骤6: 生成最终报告
echo
echo "=== 步骤6: 生成最终报告 ==="

FINAL_REPORT="$OUTPUT_DIR/FINAL_COMPARISON_REPORT.md"

cat > "$FINAL_REPORT" << EOF
# 因子图方法对比测试最终报告

## 测试配置
- **数据文件**: $(basename $DATA_FILE)
- **测试时间**: $(date)
- **输出目录**: $OUTPUT_DIR
- **数据行数**: $(wc -l < $DATA_FILE)
- **GPS有效点**: $GPS_VALID_COUNT

## 执行结果
- **传统因子图方法**: $([ $TRAD_EXIT_CODE -eq 0 ] && echo "✓ 成功" || echo "✗ 失败")
- **时间中心因子图方法**: $([ $TC_EXIT_CODE -eq 0 ] && echo "✓ 成功" || echo "✗ 失败")

## 输出文件结构
\`\`\`
$OUTPUT_DIR/
├── traditional_results/          # 传统方法结果
├── time_centric_results/         # 时间中心方法结果
├── visualizations/               # 可视化图表
│   ├── trajectory_comparison.png
│   ├── accuracy_comparison.png
│   ├── performance_comparison.png
│   ├── sensor_data_analysis.png
│   └── comparison_summary_report.md
├── comparison_report.json        # 对比结果数据
└── FINAL_COMPARISON_REPORT.md    # 本报告
\`\`\`

## 主要发现
$([ -f "$OUTPUT_DIR/visualizations/comparison_summary_report.md" ] && echo "详细分析请查看: visualizations/comparison_summary_report.md" || echo "可视化分析未完成")

## 查看结果
1. **轨迹对比图**: $OUTPUT_DIR/visualizations/trajectory_comparison.png
2. **精度对比图**: $OUTPUT_DIR/visualizations/accuracy_comparison.png  
3. **性能对比图**: $OUTPUT_DIR/visualizations/performance_comparison.png
4. **传感器数据分析**: $OUTPUT_DIR/visualizations/sensor_data_analysis.png
5. **详细报告**: $OUTPUT_DIR/visualizations/comparison_summary_report.md

## 使用建议
- 使用图片查看器打开PNG文件查看可视化结果
- 使用文本编辑器或Markdown查看器打开MD文件查看详细报告
- 原始数据和日志文件可用于进一步分析

---
报告生成时间: $(date)
EOF

echo "✓ 最终报告已生成: $FINAL_REPORT"

# 显示结果摘要
echo
echo "=== 测试完成摘要 ==="
echo "输出目录: $OUTPUT_DIR"
echo
echo "生成的文件:"
find "$OUTPUT_DIR" -type f -name "*.png" -o -name "*.md" -o -name "*.json" | sort | while read file; do
    echo "  - $(basename $file)"
done

echo
echo "查看结果的建议命令:"
echo "  cd $OUTPUT_DIR"
echo "  ls -la"
echo "  # 查看图片"
echo "  eog visualizations/*.png  # 或使用其他图片查看器"
echo "  # 查看报告"
echo "  cat visualizations/comparison_summary_report.md"

echo
echo "=== 对比测试完成 ==="
