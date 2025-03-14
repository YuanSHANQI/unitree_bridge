# unitree_bridge_readme.md    

## 0.引言:
该脚本基于ros1_bridge开源功能包编写,适用于ROS2与ROS1的桥接，用于将ROS2的topic数据转换为ROS1的topic数据，也可将ROS1的topic数据转换为ROS2的topic数据。


- **1. 支持指定话题,指定消息类型桥接**
- **2. 支持单话题,多话题桥接**
- **3. 无需过多配置环境,执行.sh文件即可完成桥接**


## 1.测试环境:
-    Noetic与Foxy同时存在的ubuntu20.04虚拟机
-    Noetic与Foxy同时存在的ubuntu20.04物理机(Orin NX)

## 2.前期准备:

### 2.1 ros1_bridge安装:
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/4b7dae5dfdbe4682afd96819f1136036.png#pic_center)

在Foxy中安装ros1_bridge:
`sudo apt install ros-foxy-ros1-bridge`


### 2.2 sh文件参数修改:

```sh
#!/bin/bash

# 1️⃣ 加载 ROS1 和 ROS2 环境
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash

# 2️⃣ 设置默认参数
TOPIC_NAMES="/a /b /c"
MESSAGE_NAMES="std_msgs.msg.String,geometry_msgs.msg.Twist,geometry_msgs.msg.Twist"

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
ros2 run ros1_bridge dynamic_bridge  &  # 后台运行
# ros2 run ros1_bridge dynamic_bridge --bridge-all-topics &  # 后台运行

# 6️⃣ 运行 Python 话题桥接
echo -e "\033[92m[INFO] 运行 topic_bridge.py...\033[0m"
python3 topic_bridge.py "$TOPIC_NAMES" "$MESSAGE_NAMES"
```
**需要修改的参数有:**
`TOPIC_NAMES="/a /b /c"`
`MESSAGE_NAMES="std_msgs.msg.String,geometry_msgs.msg.Twist,geometry_msgs.msg.Twist"`
**其中TOPIC_NAMES为需要桥接的topic名称，多个topic之间用空格隔开，MESSAGE_NAMES为需要桥接的topic对应的消息类型，多个消息类型之间用逗号隔开。**
### 2.3 查看topic的对应消息类型并修改.sh参数:
`rostopic info /topic_name`
`ros2 topic info /topic_name`

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/4c14a1fbe68141d39e062a2753487131.png#pic_center)

以这个imu数据为例(ros2):我们得到了`/unitree/imu`的消息类型为`sensor_msgs/msg/Imu`
所以我们需要将.sh参数修改为
```
TOPIC_NAMES="/unitree/imu"
MESSAGE_NAMES="sensor_msgs.msg.Imu"
```

## 3.执行.sh文件:
在unitree_bridge目录下打开终端执行:
`sudo chmod 777 * `
输入用户密码赋予文件可执行权限,然后继续输入
`./ros1_bridge.sh`

![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/b0c6c1162e9a46c0b8c2b238eb601e5c.png#pic_center)

得到以下结果,即运行成功!!!

## 4.测试桥接:
   
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/ab56fd07592d43fb93da8caf7b11e3ce.png#pic_center)

可以看到imu数据在ros1能正常显示.

## 5.注意事项:
```
ros2 run ros1_bridge dynamic_bridge  &  # 后台运行
# ros2 run ros1_bridge dynamic_bridge --bridge-all-topics &  # 后台运行
```
**`--bridge-all-topics`:
会将所有话题桥接,但是有个问题就有时候,ros2中没有ros1有的话题,启动后,ros2还是没有(这个真不好说,我的虚拟机环境是没有的,但是物理机环境却可以,但也不完全可以,有部分话题可以,有部分不可以),同理,ros1中没有ros2有的话题,启动后,ros1也没有(但一般有)
所有为了保证稳定性,还是选取不添加这个参数,ros1和ros2中都有的topic(name,msg都相同)进行桥接,这样不会出现ros1没有ros2有的话题,ros2没有ros1有的话题的情况.**
