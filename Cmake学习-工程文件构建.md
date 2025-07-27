# CMake 快速入门

## 前言

- 什么是 `CMake` ？

CMake 是一个 **跨平台的自动化构建系统工具**，主要用于生成项目的构建文件（如 Makefile、Visual Studio 工程、Ninja 等），以便编译和管理 C/C++ 等语言的代码项目。

- CMake 基本文件结构

```bash
cmake-template
├── CMakeLists.txt
└── build
└── include
└── src
     └── main.cpp
```

 `CMakeLists.txt` 文件用来告诉 CMake：项目有哪些源文件？需要哪些依赖库？生成什么目标（如可执行文件、静态库、动态库）？

- 构建系统

```bash
mkdir build
cd build
cmake ..
```

会在 `build` 目录生成平台特定的构建文件

## 例程

- 下面是宇树科技的功能包 `controller` 的 `CMakeLists.txt` 文件

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(unitree_controller)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    unitree_legged_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs 
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Declare a C++ library
add_library(${PROJECT_NAME}
    src/body.cpp 
)

add_dependencies(${PROJECT_NAME} unitree_legged_msgs_gencpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${EXTRA_LIBS}
)

# add_library(unitreeFootContactPlugin SHARED plugin/foot_contact_plugin.cc)
# target_link_libraries(unitreeFootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
# target_link_libraries(unitreeDrawForcePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(unitree_external_force src/external_force.cpp)
target_link_libraries(unitree_external_force ${catkin_LIBRARIES})

add_executable(unitree_servo src/servo.cpp)
target_link_libraries(unitree_servo ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(unitree_move_kinetic src/move_publisher.cpp)
target_link_libraries(unitree_move_kinetic ${catkin_LIBRARIES})
```

