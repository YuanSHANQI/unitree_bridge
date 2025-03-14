#!/bin/bash

# 1️⃣ 加载 ROS1 和 ROS2 环境
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash

# 2️⃣ 设置默认参数
TOPIC_NAMES="/utlidar/imu"
MESSAGE_NAMES="sensor_msgs.msg.Imu"


# 3️⃣ 解析输入参数
if [ $# -ge 1 ]; then
    TOPIC_NAMES="$1"
fi

if [ $# -ge 2 ]; then
    MESSAGE_NAMES="$2"
fi

# 4️⃣ 启动 roscore（如果尚未启动）
if ! pgrep -x "roscore" > /dev/null; then
    echo -e "\033[92m[INFO] 启动 roscore...\033[0m"
    roscore & 
    sleep 3  # 等待 roscore 启动
else
    echo -e "\033[93m[WARN] roscore 已在运行\033[0m"
fi

# 5️⃣ 启动 ros1_bridge
echo -e "\033[92m[INFO] 启动 ros1_bridge...\033[0m"
# ros2 run ros1_bridge dynamic_bridge --bridge-all-topics &  # 后台运行
ros2 run ros1_bridge dynamic_bridge &  # 后台运行

# 6️⃣ 运行 Python 话题桥接
echo -e "\033[92m[INFO] 运行 topic_bridge.py...\033[0m"
python3 topic_bridge.py "$TOPIC_NAMES" "$MESSAGE_NAMES"
