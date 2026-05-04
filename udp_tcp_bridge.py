import socket
import time
from pymavlink import mavutil

# ==========================================
# 配置参数
# ==========================================
# Gazebo 默认 HIL 端口 (根据你的 iris.sdf 或 world 文件配置)
GAZEBO_ADDR = ('127.0.0.1', 14560) 

# QEMU 监听端口 (对应你 Makefile 中的 localport)
QEMU_ADDR = ('127.0.0.1', 14540)
# QEMU 发送端口 (对应你 Makefile 中的 port)
QEMU_SEND_PORT = 14580

def start_bridge():
    # 1. 创建与 Gazebo 通信的 UDP Socket
    gazebo_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gazebo_sock.bind(('127.0.0.1', 14560)) # 绑定本地端口接收 Gazebo 数据
    gazebo_sock.setblocking(False)

    # 2. 创建与 QEMU 通信的 UDP Socket
    qemu_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    qemu_sock.bind(('127.0.0.1', QEMU_SEND_PORT)) # 接收来自 QEMU 的指令
    qemu_sock.setblocking(False)

    print(f"🚀 直接 HIL 网桥已启动")
    print(f"📡 Gazebo 链路: {GAZEBO_ADDR}")
    print(f"📟 QEMU 链路: localhost:{QEMU_SEND_PORT} -> {QEMU_ADDR}")

    last_heartbeat = 0
    
    # 存储 Gazebo 的实际地址（因为 Gazebo 端口可能动态变化）
    actual_gazebo_addr = GAZEBO_ADDR 

    while True:
        # --- A. 从 Gazebo 转发数据到 QEMU ---
        try:
            data, addr = gazebo_sock.recvfrom(2048)
            actual_gazebo_addr = addr # 更新 Gazebo 实际地址
            # 原样转发 HIL_SENSOR 消息给 QEMU
            qemu_sock.sendto(data, QEMU_ADDR)
        except BlockingIOError:
            pass

        # --- B. 从 QEMU 转发指令到 Gazebo ---
        try:
            data, addr = qemu_sock.recvfrom(2048)
            # 原样转发 HIL_ACTUATOR_CONTROLS 消息给 Gazebo
            # 这步非常关键，它会触发 Gazebo 的物理步进 (Lockstep)
            gazebo_sock.sendto(data, actual_gazebo_addr)
        except BlockingIOError:
            pass

        # --- C. 辅助：网桥自身心跳 (可选) ---
        now = time.time()
        if now - last_heartbeat > 1.0:
            # 可以在这里打印流量监控
            last_heartbeat = now

        # 防止死循环吃满 CPU，但在 HIL 模式下这个延迟要极低
        time.sleep(0.0001) 

if __name__ == "__main__":
    try:
        start_bridge()
    except KeyboardInterrupt:
        print("\n🛑 网桥已关闭")