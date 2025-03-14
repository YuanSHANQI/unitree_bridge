#!/usr/bin/env python3

import sys
import rospy
import importlib

def import_message_types(message_names):
    """
    动态导入 ROS 消息类型
    :param message_names: 消息类型字符串（例如："std_msgs.msg.String,geometry_msgs.msg.Twist"）
    :return: 消息类型列表 [std_msgs.msg.String, geometry_msgs.msg.Twist]
    """
    message_list = []
    for msg_type in message_names.split(","):  # 逗号分割
        msg_type = msg_type.strip()  # 去除空格
        if "." not in msg_type or msg_type.count(".") < 2:
            print(f"\033[91m[ERROR] 无效的消息类型格式: {msg_type}\033[0m")
            sys.exit(1)

        try:
            package, subpackage, msg_name = msg_type.rsplit(".", 2)  # 确保拆分成 3 部分
            module = importlib.import_module(f"{package}.{subpackage}")  # 导入模块
            message_list.append(getattr(module, msg_name))  # 获取类
        except Exception as e:
            print(f"\033[91m[ERROR] 无法导入消息类型 {msg_type}: {e}\033[0m")
            sys.exit(1)

    return message_list

if __name__ == "__main__":
    rospy.init_node("topic_bridge")

    if len(sys.argv) < 3:
        print("\033[91mUsage: topic_bridge.py '<topic1> <topic2> ...' '<msg_type1,msg_type2,...>'\033[0m")
        sys.exit(1)

    topic_list = sys.argv[1].split()
    message_names = sys.argv[2]

    message_types = import_message_types(message_names)

    if len(topic_list) != len(message_types):
        print("\033[91m[ERROR] 话题数量与消息类型数量不匹配！\033[0m")
        sys.exit(1)

    print("\033[92m!!! ROS~ROS2 SUCCESS !!!\033[0m")

    publishers = {}

    for topic, msg_type in zip(topic_list, message_types):
        publishers[topic] = rospy.Publisher(topic, msg_type, queue_size=10)
        print(f"\033[94m[INFO] 话题 {topic} -> {msg_type.__name__} 已创建\033[0m")

    rospy.spin()
