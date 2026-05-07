#!/bin/bash

# ==========================================
# CEMU 项目 - HIL 仿真一键启动脚本 (MAVProxy 桥接优化版)
# ==========================================

# 1. 路径配置
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORLD_PATH="$ROOT_DIR/gazebo/iris_irlock.world"
PX4_MODELS_DIR="$ROOT_DIR/gazebo/models"
PX4_PLUGIN_DIR="$ROOT_DIR/gazebo/build_gazebo-classic"

# ==========================================
# 🛑 终极清理函数 (捕捉各种退出信号)
# ==========================================
function cleanup() {
    echo ""
    echo "🛑 收到退出指令，正在强杀所有仿真进程..."
    
    # 1. 杀掉通过 PID 记录的进程
    kill -9 $MAVPROXY_PID $GAZEBO_PID 2>/dev/null || true
    
    # 2. 无差别强杀残留的同名进程 (防止僵尸进程)
    killall -9 gzserver gzclient qemu-system-arm mavproxy.py 2>/dev/null || true
    
    # 3. 清理生成的临时文件
    rm -f ./cemu_iris.sdf
    
    echo "✅ 所有进程已彻底清理，环境安全！"
    exit 0
}

# 绑定信号：当收到 Ctrl+C(INT)、被强制终止(TERM) 或 脚本自然结束(EXIT) 时，立刻执行 cleanup 函数
trap cleanup SIGINT SIGTERM EXIT

# ==========================================
# 正常启动流程
# ==========================================

echo "🧹 [1/6] 发车前环境清理..."
killall -9 gzserver gzclient qemu-system-arm mavproxy.py 2>/dev/null || true
sleep 1

# 开启“遇错即停”模式
set -e 

# --- 注入环境变量 ---
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PX4_MODELS_DIR
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$PX4_PLUGIN_DIR

echo "🔨 [2/6] 编译 CEMU 飞控固件..."
make clean
make -j$(nproc)

echo "🌍 [3/6] 启动 Gazebo 物理环境..."
gazebo --verbose "$WORLD_PATH" &
GAZEBO_PID=$!

echo "⏳ 等待 Gazebo 加载场地 (5秒)..."
sleep 3

echo "🔧 [4/6] 动态生成专属的 UDP 版无人机模型..."
cp "$PX4_MODELS_DIR/iris/iris.sdf" ./cemu_iris.sdf
# 强制关闭 SDF 模板中的 TCP 模式，让 Gazebo 走 UDP 14560
sed -i 's/<use_tcp>1<\/use_tcp>/<use_tcp>0<\/use_tcp>/g' ./cemu_iris.sdf
sed -i 's/<mavlink_udp_port>.*<\/mavlink_udp_port>/<mavlink_udp_port>14560<\/mavlink_udp_port>/g' ./cemu_iris.sdf

echo "🚁 [5/6] 正在向场地投放定制版 Iris 无人机..."
gz model --spawn-file="./cemu_iris.sdf" --model-name=iris -x 0 -y 0 -z 0.1
sleep 2

# === 关键修改：启动 MAVProxy 作为工业级路由 ===
echo "🌉 [6/6] 启动 MAVProxy 进行 UDP/TCP 桥接..."
# 逻辑: 接收 Gazebo 发往本地 14560 的 UDP 数据，并以 TCP 客户端模式连入 QEMU 的 14580 端口
mavproxy.py --master=udp:127.0.0.1:14560 --out=tcp:127.0.0.1:14580 --daemon &
MAVPROXY_PID=$!
sleep 2

echo "🚀 [启动完成] 运行 QEMU 并接入 HIL 链路..."
echo "💡 提示: 退出仿真请按 【Ctrl + A】松开后按 【X】"
echo "------------------------------------------------"

# 执行 QEMU 启动 (注意：Makefile 中的 run 目标必须配置为 TCP 服务端: -chardev socket,server=on,wait=off...)
make run