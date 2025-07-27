# ROS2 实现深蓝学院宇树机器人仿真（失败）

## 环境必备

 深蓝学院的作业是使用 `ROS` 一件安装指令，但我们是在自己的笔记本上已经安装过 `ROS2`，因此我们跳过此流程，进行自主安装，若中间缺少相关依赖自行补齐，下面记录此过程。

- 首先更新系统，在终端执行指令

```bash
sudo apt update && sudo apt upgrade -y
```

- 按照深蓝指南安装依赖库

```bash
sudo apt install -y \
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

- 添加 `Gazebo` 官方软件源

```bash
sudo apt install -y wget
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

发生权限不够的终端响应：

<img src="/home/bohao/.config/Typora/typora-user-images/image-20250713184405095.png" alt="image-20250713184405095" style="zoom: 50%;" />

尝试将 `wget https://packages.osrfoundation.org/gazebo.gpg` 前加上管理员权限 `sudo`，成功运行

- 安装 `Gazebo` 

```bash
sudo apt update
sudo apt install -y gazebo11 libgazebo11-dev
```

报错

<img src="/home/bohao/.config/Typora/typora-user-images/image-20250713185111675.png" alt="image-20250713185111675" style="zoom:50%;" />

分析得知，我本电脑使用的是较新的 Ubuntu 版本（ 22.04），其默认已不支持 Gazebo 11（支持 Ignition Gazebo 或 Gazebo Garden 等新版本），因此需要安装符合新版本的 Gazebo

```bash
sudo apt install ros-humble-gazebo-*
```

- 尝试开启 Gazebo

```bash
gazebo worlds/empty.world
```

<img src="/home/bohao/.config/Typora/typora-user-images/image-20250714003724092.png" alt="image-20250714003724092" style="zoom:33%;" />

运行成功

- 安装 C++ 数值计算库 Eigen3.4 

```bash
cd ./eigen-3.4.0/ # 在工作空间中切换到 eigen-3.4.0 文件夹
rm -r build # 原来有 build 文件先删除
mkdir build
cd build # 创建 build 文件夹并进入
cmake ..
sudo make install
```

- 在工作空间直接进行编译，看看会报错什么

```bash
cd ..
cd ..
colcon build
```

<img src="/home/bohao/.config/Typora/typora-user-images/image-20250714011037063.png" alt="image-20250714011037063" style="zoom: 50%;" />

由于 `colcon build` 是 ROS2 的构建命令，而深蓝项目是基于 ROS1 开发的，主要是 `catkin` 来构建，因此不兼容。
