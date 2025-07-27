# ROS 实现深蓝学院宇树机器人仿真

## 环境必备

 深蓝学院的作业是使用 `ROS` 一件安装指令，在终端进行操作

- 使用鱼香 ROS 一键安装指令

```bash
$ wget http://fishros.com/install -O fishros && . fishros
```

此时安装后的版本为 ROS-Noetic

- 更新系统，在终端执行指令

```bash
$ sudo apt update && sudo apt upgrade -y
```

- 按照深蓝指南安装与 ROS 相关的依赖库

```bash
$ sudo apt install -y \
    cmake \
    curl \
    git \
    libfreeimage-dev \
    libprotoc-dev \
    protobuf-compiler \
    libignition-math6-dev \
    libsqlite3-dev \
    libtinyxml2-dev \
    libgflags-dev \
    libavformat-dev \
    libavcodec-dev
```

- 添加 `Gazebo` 官方软件源，为安装其做准备

```bash
$ sudo apt install -y wget
$ wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

- 安装 Gazebo，首先更新软件再进行安装

```bash
$ sudo apt update
$ sudo apt install -y gazebo11 libgazebo11-dev
```

- 配置 Gazebo 的环境变量，告诉系统此文件的路径

```bash
$ echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
$ source ~/.bashrc
```

- 验证其安装结果，初始化软件

```bash
$ gazebo worlds/empty.world
```

这时候由于环境变量配置了，就每次添加环境变量

![image-20250714152914511](https://cdn.jsdelivr.net/gh/zhaobohao-byte/img@main/image-20250714152914511.png)

- 安装 Eigen 数值计算库，为 Cpp 的数值计算作准备

```bash
$ cd robocup_g1/eigen-3.4.0
$ mkdir build
$ cd build
$ cmake ..
$ sudo make install
```

- 安装 RealSense SDK，用于摄像头等外设与 ROS 进行通讯

```bash
$ sudo apt-get install ros-noetic-realsense2-camera
```

- 安装 Ignition-math ，作为 Gazebo 的数字计算库

```bash
$ sudo apt install libignition-math4-dev
```

## 编译和运行

找到项目工作空间的根目录，通常根目录下包含文件 `.catkin_workspace`，用于标识工作空间，但是往往不会直接显示，按住组合键 `ctrl`+`H`显示隐藏文件，如下

<img src="/home/bohao/.config/Typora/typora-user-images/image-20250714153746604.png" alt="image-20250714153746604" style="zoom: 80%;" />

- 在此处打开终端，使用 ROS 专属的构建命令

```bash
$ conda activate ros_env   # 我使用的系统装有 conda，需要切换到独有的环境中支持 python3.8
$ catkin_make
```

- 启动 ROS 工作节点，先启动 Gazebo 仿真节点

深蓝给出的启动方式为

```bash
$ source ~/robocup_g1/devel/setup.bash
$ roslaunch unitree_guide gazebo.launch
```

第一个命令是临时地将此环境变量告诉给系统，每次输入这个有点麻烦，我们做一个一劳永逸的工作

先找到当前工作空间的 `devel/setup.bash` 的绝对路径

```bash
$ pwd
```

 得到终端的响应

```bash
/home/robot/RemoteUsers/Bohao/examples/robocup_g1/devel
```

再在当前工作空间打开终端，并输入命令

```bash
$ source ./devel/setup.bash # 仅在当前终端生效
$ echo " source ~/RemoteUsers/Bohao/examples/robocup_g1/devel/setup.bash" >> ~/.bashrc # 所有终端均生效
```

这时候就将环境变量加入了所有的终端中，再新起一个终端后，直接输入指令

```bash
$ roslaunch unitree_guide gazebo.launch
```

![image-20250714161118671](https://cdn.jsdelivr.net/gh/zhaobohao-byte/img@main/image-20250714161118671.png)

成功启动了 Gazebo 仿真节点

- 在工作空间内新建一个终端

```bash
$ conda activate ros_env   # 激活我的 ros 环境
$ ./devel/lib/unitree_guide/junior_ctrl
```

此时不用配置环境变量了，因为我们此前将环境变量便携的一次性配置好了，进入键盘控制节点

- 在 `junior_ctrl` 主界面输入 `2`，控制机器人从 **State_Passive** 切换到 **fixed stand**

- 回到 Gazebo 主界面，按下暂停键，然后在主菜单中选择 **Edit/Reset Model Poses** 以重置机器人的位姿

- 在 `junior_ctrl` 主界面输入 `4`，控制机器人从 **fixed stand** 切换到 **LOCOMOTION**

- 回到 Gazebo 主界面，点击播放键，重启应用

运动的键盘控制为

- **前后运动**：
  - **`W` 键**：向前运动
  - **`S` 键**：向后运动

- **左右平移**：
  - **`A` 键**：向左平移
  - **`D` 键**：向右平移

- **左右旋转**：
  - **`J` 键**：向左旋转
  - **`L` 键**：向右旋转

![image-20250714161818535](https://cdn.jsdelivr.net/gh/zhaobohao-byte/img@main/image-20250714161818535.png)

即可进行机器人的畅玩了

## 题外，ROS 节点运行情况

- 安装 rqt 小工具，对应适配的 ROS-noetic 版本

```bash
$ sudo apt install ros-noetic-rqt
```

- 运行 rqt 

```bash
$ rqt
```

观察到运行中的节点情况为

<img src="https://cdn.jsdelivr.net/gh/zhaobohao-byte/img@main/image-20250714163821926.png" alt="image-20250714163821926" style="zoom: 50%;" />

话题激活情况为

![rosgraph](https://cdn.jsdelivr.net/gh/zhaobohao-byte/img@main/rosgraph.png)

方便后续进行分析和学习
