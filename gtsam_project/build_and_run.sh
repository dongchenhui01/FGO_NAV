#!/bin/bash

# AUV因子图导航算法编译和运行脚本

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 获取脚本目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# 检查GTSAM是否安装
check_gtsam() {
    log_info "检查GTSAM安装..."
    
    if [ ! -d "/usr/local/include/gtsam" ] && [ ! -d "/usr/include/gtsam" ]; then
        log_error "GTSAM未找到，请先安装GTSAM库"
        exit 1
    fi
    
    log_success "GTSAM已安装"
}

# 编译项目
build_project() {
    log_info "编译AUV导航算法..."
    
    # 创建build目录
    mkdir -p build
    cd build
    
    # CMake配置
    cmake .. -DCMAKE_BUILD_TYPE=Release
    
    # 编译
    make -j$(nproc)
    
    if [ -f "bin/auv_navigation" ]; then
        log_success "编译成功！"
    else
        log_error "编译失败！"
        exit 1
    fi
    
    cd ..
}

# 运行算法
run_algorithm() {
    local csv_file="$1"
    
    if [ -z "$csv_file" ]; then
        log_error "请提供CSV数据文件路径"
        echo "用法: $0 <csv_file>"
        exit 1
    fi
    
    if [ ! -f "$csv_file" ]; then
        log_error "CSV文件不存在: $csv_file"
        exit 1
    fi
    
    log_info "运行AUV导航算法..."
    log_info "输入文件: $csv_file"
    
    # 创建结果目录
    mkdir -p results
    
    # 运行算法
    ./build/bin/auv_navigation "$csv_file"
    
    if [ -f "results/trajectory.csv" ]; then
        log_success "算法运行完成！"
        log_info "轨迹结果保存在: results/trajectory.csv"
    else
        log_error "算法运行失败，未生成轨迹文件"
        exit 1
    fi
}

# 可视化结果
visualize_results() {
    local csv_file="$1"
    
    log_info "生成轨迹可视化..."
    
    # 检查Python依赖
    python3 -c "import matplotlib, pandas, numpy" 2>/dev/null || {
        log_warning "缺少Python依赖，尝试安装..."
        pip3 install matplotlib pandas numpy
    }
    
    # 设置脚本权限
    chmod +x src/visualize_trajectory.py
    
    # 运行可视化
    if [ -n "$csv_file" ]; then
        python3 src/visualize_trajectory.py \
            --trajectory results/trajectory.csv \
            --gps_ref "$csv_file" \
            --output_dir results
    else
        python3 src/visualize_trajectory.py \
            --trajectory results/trajectory.csv \
            --output_dir results
    fi
    
    log_success "可视化完成！结果保存在 results/ 目录"
}

# 主函数
main() {
    log_info "AUV因子图导航算法 - 编译和运行"
    
    # 检查参数
    if [ $# -lt 1 ]; then
        echo "用法: $0 <csv_file> [options]"
        echo ""
        echo "选项:"
        echo "  --build-only    仅编译，不运行"
        echo "  --run-only      仅运行，不编译"
        echo "  --no-viz        不生成可视化"
        echo ""
        echo "示例:"
        echo "  $0 ../underwater_nav_fgo/data/sensor_fusion_log_20250712_102606.csv"
        echo "  $0 data.csv --build-only"
        exit 1
    fi
    
    local csv_file="$1"
    local build_only=false
    local run_only=false
    local no_viz=false
    
    # 解析选项
    shift
    while [[ $# -gt 0 ]]; do
        case $1 in
            --build-only)
                build_only=true
                shift
                ;;
            --run-only)
                run_only=true
                shift
                ;;
            --no-viz)
                no_viz=true
                shift
                ;;
            *)
                log_error "未知选项: $1"
                exit 1
                ;;
        esac
    done
    
    # 执行步骤
    check_gtsam
    
    if [ "$run_only" = false ]; then
        build_project
    fi
    
    if [ "$build_only" = false ]; then
        run_algorithm "$csv_file"
        
        if [ "$no_viz" = false ]; then
            visualize_results "$csv_file"
        fi
    fi
    
    log_success "所有任务完成！"
}

# 运行主函数
main "$@"
