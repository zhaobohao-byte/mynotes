# ROS 运行管理

## 前言

在多级层深的ROS系统中，其实现与维护可能会出现一些问题，比如，如何关联不同的功能包，繁多的ROS节点应该如何启动？功能包、节点、话题、参数重名时应该如何处理？不同主机上的节点如何通信？

本章主要内容介绍在ROS中上述问题的解决策略，预期达成学习目标也与上述问题对应：

- 掌握元功能包使用语法；
- 掌握launch文件的使用语法；
- 理解什么是ROS工作空间覆盖，以及存在什么安全隐患；
- 掌握节点名称重名时的处理方式；
- 掌握话题名称重名时的处理方式；
- 掌握参数名称重名时的处理方式；
- 能够实现ROS分布式通信。

## 元功能包

- 问题引入

完成ROS中一个系统性的功能，可能涉及到多个功能包，比如实现了机器人导航模块，该模块下有地图、定位、路径规划...等不同的子级功能包。那么调用者安装该模块时，需要逐一的安装每一个功能包吗？

显而易见的，逐一安装功能包的效率低下，在ROS中，提供了一种方式可以将不同的功能包打包成一个功能包，当安装某个功能模块时，直接调用打包后的功能包即可，该包又称之为元功能包(metapackage)。

- 概念

MetaPackage是Linux的一个文件管理系统的概念。是ROS中的一个虚包，里面没有实质性的内容，但是它依赖了其他的软件包，通过这种方法可以把其他包组合起来，我们可以认为它是一本书的目录索引，告诉我们这个包集合中有哪些子包，并且该去哪里下载。

例如：

`sudo apt install ros-noetic-desktop-full` 命令安装 `ros` 时就使用了元功能包，该元功能包依赖于ROS中的其他一些功能包，安装该包时会一并安装依赖。

- 作用

方便用户的安装，我们只需要这一个包就可以把其他相关的软件包组织到一起安装了。

- 实现

**首先:**新建一个功能包

```bash
# 当前工作区
catkin_create new_pkg
```

**然后:**修改**package.xml** ,内容如下:

```xml
 <exec_depend>new_pkg</exec_depend>
 .....
 <export>
   <metapackage />
 </export>
```

**最后:**修改 CMakeLists.txt,内容如下:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(demo)
find_package(catkin REQUIRED)
catkin_metapackage()
```

## 多节点运行launch

- 问题引入

一个程序中可能需要启动多个节点，比如:ROS 内置的小乌龟案例，如果要控制乌龟运动，要启动多个窗口，分别启动 roscore、乌龟界面节点、键盘控制节点。如果每次都调用 rosrun 逐一启动，显然效率低下，如何优化?

- 概念

launch 文件是一个 XML 格式的文件，可以启动本地和远程的多个节点，还可以在参数服务器中设置参数

其能简化节点的配置与启动，提高ROS程序的启动效率。

- 使用示例

以 turtlesim 为例演示

新建launch文件

在功能包下添加 launch目录, 目录下新建 xxxx.launch 文件，编辑 launch 文件

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node"     name="myTurtle" output="screen" />
    <node pkg="turtlesim" type="turtle_teleop_key"  name="myTurtleContro" output="screen" />
</launch>
```

调用 launch 文件

```bash
roslaunch 包名 xxx.launch
```

**注意:** `roslaunch` 命令执行 `launch` 文件时，首先会判断是否启动了 `roscore`，如果启动了，则不再启动，否则，会自动调用 `roscore`

| 标签                 | 出现层级                  | 主要作用                        | 常用属性/子标签                                              | 备注                             |                                                  |                          |
| -------------------- | ------------------------- | ------------------------------- | ------------------------------------------------------------ | -------------------------------- | ------------------------------------------------ | ------------------------ |
| `<launch>`           | 根节点                    | 整个 launch 文件的容器          | `deprecated="reason"`                                        | 每个 launch 文件有且只有一个     |                                                  |                          |
| `<node>`             | `<launch>` 内             | 启动一个 ROS 节点               | `pkg`, `type`, `name`, `args`, `output`, `respawn`, `required`, `clear_params`, `cwd`, `launch-prefix` | 最常用标签                       |                                                  |                          |
| `<param>`            | `<launch>` 或 `<node>` 内 | 向 Parameter Server 设置参数    | `name`, `value`, `type`, `textfile`, `binfile`, `command`    | 写在 `<node>` 内时作用域为该节点 |                                                  |                          |
| `<rosparam>`         | `<launch>` 或 `<node>` 内 | 批量加载 YAML/字典参数          | \`command="load                                              | dump                             | delete"`, `file`, `param`, `ns`, `subst\_value\` | 推荐用 YAML 管理大量参数 |
| `<arg>`              | `<launch>` 内             | 声明/定义 launch 文件级输入参数 | `name`, `default`, `value`                                   | 通过 `$(arg xxx)` 在文件内引用   |                                                  |                          |
| `<remap>`            | `<launch>` 或 `<node>` 内 | 话题/服务名重映射               | `from`, `to`                                                 | 作用域遵循放置位置               |                                                  |                          |
| `<group>`            | `<launch>` 内             | 逻辑分组，可加命名空间          | `ns`, `clear_params`, `if`, `unless`                         | 常用于多机器人/多实例            |                                                  |                          |
| `<include>`          | `<launch>` 内             | 复用其它 launch 文件            | `file`, `ns`, `pass_all_args="true"`                         | 可配合 `<arg>` 传参              |                                                  |                          |
| `<env>`              | `<launch>` 或 `<node>` 内 | 设置环境变量                    | `name`, `value`                                              | 仅对启动的进程生效               |                                                  |                          |
| `<machine>`          | `<launch>` 内             | 声明远程机器                    | `name`, `address`, `user`, `password`, `env-loader`, `default` | 多机 roslaunch 时使用            |                                                  |                          |
| `<test>`             | `<launch>` 内             | 启动 rostest 测试节点           | `test-name`, `pkg`, `type`, `time-limit`, `args`             | 仅在 `rostest` 命令下生效        |                                                  |                          |
| `<master>`           | `<launch>` 内             | 配置 ROS Master URI             | `uri`                                                        | 极少用，通常由环境变量指定       |                                                  |                          |
| `if` / `unless` 属性 | 所有元素                  | 条件判断                        | `if="$(arg foo)"`, `unless="$(arg bar)"`                     | 布尔值或 0/1                     |                                                  |                          |

`<node>`标签用于指定 ROS 节点，是最常见的标签，需要注意的是: roslaunch 命令不能保证按照 node 的声明顺序来启动节点(节点的启动是多进程的)

### node标签

#### 属性

- pkg="包名"

	节点所属的包

- type="nodeType"

	节点类型(与之相同名称的可执行文件)

- name="nodeName"

	节点名称(在 ROS 网络拓扑中节点的名称)

- args="xxx xxx xxx" (可选)

	将参数传递给节点

- machine="机器名"

	在指定机器上启动节点

- respawn="true | false" (可选)

	如果节点退出，是否自动重启

- respawn_delay=" N" (可选)

	如果 respawn 为 true, 那么延迟 N 秒后启动节点

- required="true | false" (可选)

	该节点是否必须，如果为 true,那么如果该节点退出，将杀死整个 roslaunch

- ns="xxx" (可选)

	在指定命名空间 xxx 中启动节点

- clear_params="true | false" (可选)

	在启动前，删除节点的私有空间的所有参数

- output="log | screen" (可选)

	日志发送目标，可以设置为 log 日志文件，或 screen 屏幕,默认是 log

#### 子级标签

- env 环境变量设置
- remap 重映射节点名称
- rosparam 参数设置
- param 参数设置

### include 标签

`include`标签用于将另一个 xml 格式的 launch 文件导入到当前文件

#### 属性

- file="$(find 包名)/xxx/xxx.launch"

	要包含的文件路径

- ns="xxx" (可选)

	在指定命名空间导入文件

#### 子级标签

- env 环境变量设置
- arg 将参数传递给被包含的文件

### remap 标签

用于话题重命名

#### 属性

- from="xxx"

	原始话题名称

- to="yyy"

	目标名称

#### 子级标签

- 无

### param标签

`<param>`标签主要用于在参数服务器上设置参数，参数源可以在标签中通过 value 指定，也可以通过外部文件加载，在`<node>`标签中时，相当于私有命名空间。

#### 属性

- name="命名空间/参数名"

	参数名称，可以包含命名空间

- value="xxx" (可选)

	定义参数值，如果此处省略，必须指定外部文件作为参数源

- type="str | int | double | bool | yaml" (可选)

	指定参数类型，如果未指定，roslaunch 会尝试确定参数类型，规则如下:

	- 如果包含 '.' 的数字解析未浮点型，否则为整型
	- "true" 和 "false" 是 bool 值(不区分大小写)
	- 其他是字符串

#### 子级标签

- 无

### rosparam 标签

`<rosparam>`标签可以从 YAML 文件导入参数，或将参数导出到 YAML 文件，也可以用来删除参数，`<rosparam>`标签在`<node>`标签中时被视为私有。

#### 属性

- command="load | dump | delete" (可选，默认 load)

	加载、导出或删除参数

- file="$(find xxxxx)/xxx/yyy...."

	加载或导出到的 yaml 文件

- param="参数名称"

- ns="命名空间" (可选)

#### 子级标签

- 无

### group 标签

`<group>`标签可以对节点分组，具有 ns 属性，可以让节点归属某个命名空间

#### 属性

- ns="名称空间" (可选)

- clear_params="true | false" (可选)

	启动前，是否删除组名称空间的所有参数(慎用....此功能危险)

#### 子级标签

- 除了launch 标签外的其他标签

### arg 标签

`<arg>`标签是用于动态传参，类似于函数的参数，可以增强launch文件的灵活性

#### 属性

- name="参数名称"

- default="默认值" (可选)

- value="数值" (可选)

	不可以与 default 并存

- doc="描述"

	参数说明

#### 子级标签

- 无

#### 示例

- launch文件传参语法实现，hello.lcaunch

	```xml
	<launch>
	    <arg name="xxx" />
	    <param name="param" value="$(arg xxx)" />
	</launch>
	```

命令行调用launch传参

```bash
roslaunch hello.launch xxx:=值
```

## ROS工作空间覆盖

- 前言

ROS 开发中，会自定义工作空间且自定义工作空间可以同时存在多个，可能会出现一种情况:  虽然特定工作空间内的功能包不能重名，但是自定义工作空间的功能包与内置的功能包可以重名或者不同的自定义的工作空间中也可以出现重名的功能包，那么调用该名称功能包时，会调用哪一个呢？比如：自定义工作空间A存在功能包 turtlesim，自定义工作空间B也存在功能包  turtlesim，当然系统内置空间也存在turtlesim，如果调用turtlesim包，会调用哪个工作空间中的呢？

- 测试

新建工作空间A与工作空间B，两个工作空间中都创建功能包: turtlesim。

1.在 `~/.bashrc` 文件下**追加**当前工作空间的 `bash` 格式如下:

```
source /home/用户/路径/工作空间A/devel/setup.bash
source /home/用户/路径/工作空间B/devel/setup.bash
```

2.新开命令行:`source .bashrc`加载环境变量

3.查看ROS环境环境变量`echo $ROS_PACKAGE_PATH`

结果:自定义工作空间B:自定义空间A:系统内置空间

4.调用命令:`roscd turtlesim`会进入自定义工作空间B

- 分析

ROS 会解析 `.bashrc` 文件，并生成 `ROS_PACKAGE_PATH ROS` 包路径，该变量中按照 `.bashrc` 中配置设置工作空间优先级，在设置时需要遵循一定的原则:`ROS_PACKAGE_PATH` 中的值，和 `.bashrc`  的配置顺序相反--->后配置的优先级更高，如果更改自定义空间A与自定义空间B的`source`顺序，那么调用时，将进入工作空间A。

其中 `setup.bash` 是 `ROS1/2` 在编译之后自动生成的一段 shell 脚本，主要目的是

| 动作                                           | 具体效果                                                     |
| ---------------------------------------------- | ------------------------------------------------------------ |
| 设置 `ROS_PACKAGE_PATH`                        | 把你的工作空间 `src` 目录追加进去，让 `rospack find xxx` 能找到新包。 |
| 设置 `CMAKE_PREFIX_PATH`                       | 让以后在同一个工作空间或别的空间编译时，CMake 能找到这里已经编译好的库/头文件。 |
| 设置 `PATH` / `LD_LIBRARY_PATH` / `PYTHONPATH` | 把 `devel/lib`、`devel/bin`、`devel/lib/pythonX.Y/dist-packages` 等目录塞进去，保证运行/调试可执行文件、Python 节点时不会找不到库。 |
| source 上一级空间（如果有）                    | 如果该工作空间是 `overlay`，它会再 `source` 你之前已 `source` 过的 `underlay` 的 `setup.bash`，保证链式依赖。 |

- 结论

功能包重名时，会按照 ROS_PACKAGE_PATH 查找，配置在前的会优先执行。

## 节点重命名

- 问题引入

ROS 中创建的节点是有名称的，C++初始化节点时通过API:`ros::init(argc,argv,"xxxx");`来定义节点名称，在Python中初始化节点则通过 `rospy.init_node("yyyy")`  来定义节点名称。在ROS的网络拓扑中，是不可以出现重名的节点的，因为假设可以重名存在，那么调用时会产生混淆，这也就意味着，不可以启动重名节点或者同一个节点启动多次，的确，在ROS中如果启动重名节点的话，之前已经存在的节点会被直接关闭，但是如果有这种需求的话，怎么优化呢？

### rosrun设置命名空间

- 语法: rosrun 包名 节点名 __ns:=新名称

```bash
rosrun turtlesim turtlesim_node __ns:=/xxx
rosrun turtlesim turtlesim_node __ns:=/yyy
```

两个节点都可以正常运行

- 运行结果

`rosnode list`查看节点信息,显示结果:

```bash
/xxx/turtlesim
/yyy/turtlesim
```

### rosrun名称重映射

- 语法: rosrun 包名 节点名 __name:=新名称

```bash
rosrun turtlesim  turtlesim_node __name:=t1 |  rosrun turtlesim   turtlesim_node /turtlesim:=t1(不适用于python)
rosrun turtlesim  turtlesim_node __name:=t2 |  rosrun turtlesim   turtlesim_node /turtlesim:=t2(不适用于python)
```

两个节点都可以运行

- 运行结果

`rosnode list`查看节点信息,显示结果:

```bash
/t1
/t2
```

### rosrun设置命名空间同时名称重映射

- 语法: rosrun 包名 节点名 __ns:=新名称 __name:=新名称

```bash
rosrun turtlesim turtlesim_node __ns:=/xxx __name:=tn
```

- 运行结果

`rosnode list`查看节点信息,显示结果:

```bash
/xxx/tn
```

### roslaunch设置

介绍 launch 文件的使用语法时，在 node 标签中有两个属性: name 和 ns，二者分别是用于实现名称重映射与命名空间设置的。使用launch文件设置命名空间与名称重映射也比较简单。

- launch文件

```xml
<launch>

    <node pkg="turtlesim" type="turtlesim_node" name="t1" />
    <node pkg="turtlesim" type="turtlesim_node" name="t2" />
    <node pkg="turtlesim" type="turtlesim_node" name="t1" ns="hello"/>

</launch>
```

在 node 标签中，name 属性是必须的，ns 可选。

- 运行

`rosnode list`查看节点信息,显示结果:

```
/t1
/t2
/t1/hello
```

### 编码设置

如果自定义节点实现，那么可以更灵活的设置命名空间与重映射实现。

- C++ 实现:重映射

核心代码:`ros::init(argc,argv,"zhangsan",ros::init_options::AnonymousName);`

执行会在名称后面添加时间戳。

- C++ 实现:命名空间

核心代码

```cpp
  std::map<std::string, std::string> map;
  map["__ns"] = "xxxx";
  ros::init(map,"wangqiang");
```

执行节点名称设置了命名空间。

---

## ROS话题名称设置

### rosrun设置

**rosrun名称重映射语法: rorun 包名 节点名 话题名:=新话题名称**

实现teleop_twist_keyboard与乌龟显示节点通信方案由两种：

- 方案1

将 teleop_twist_keyboard 节点的话题设置为`/turtle1/cmd_vel`

启动键盘控制节点:`rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/turtle1/cmd_vel`

启动乌龟显示节点: `rosrun turtlesim turtlesim_node`

二者可以实现正常通信

- 方案2

将乌龟显示节点的话题设置为 `/cmd_vel`

启动键盘控制节点:`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

启动乌龟显示节点: `rosrun turtlesim turtlesim_node /turtle1/cmd_vel:=/cmd_vel`

二者可以实现正常通信

### launch设置

**launch 文件设置话题重映射语法:**

```xml
<node pkg="xxx" type="xxx" name="xxx">
    <remap from="原话题" to="新话题" />
</node>
```

实现 `teleop_twist_keyboard` 与乌龟显示节点通信方案由两种：

- 方案1

将 `teleop_twist_keyboard` 节点的话题设置为`/turtle1/cmd_vel`

```xml
<launch>

    <node pkg="turtlesim" type="turtlesim_node" name="t1" />
    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="key">
        <remap from="/cmd_vel" to="/turtle1/cmd_vel" />
    </node>

</launch>
```

二者可以实现正常通信

- 方案2

将乌龟显示节点的话题设置为 `/cmd_vel`

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="t1">
        <remap from="/turtle1/cmd_vel" to="/cmd_vel" />
    </node>
    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="key" />

</launch>
```

二者可以实现正常通信

### 编码设置话题

演示准备:

1.初始化节点设置一个节点名称

```cpp
ros::init(argc,argv,"hello")
```

2.设置不同类型的话题

3.启动节点时，传递一个 __ns:= xxx

4.节点启动后，使用 rostopic 查看话题信息

- 全局名称

**格式:**以`/`开头的名称，和节点名称无关

**比如:**/xxx/yyy/zzz，适用于跨节点通信，确保名称唯一

**示例1:**`ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter",1000);`

**结果1:**`/chatter`

**示例2:**`ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter/money",1000);`

**结果2:**`/chatter/money`

- 相对名称

**格式:**非`/`开头的名称,参考命名空间(与节点名称平级)来确定话题名称，适用于节点内部通信，避免硬编码全局路径

**示例1:**`ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",1000);`

**结果1:**`xxx/chatter`

**示例2:**`ros::Publisher pub = nh.advertise<std_msgs::String>("chatter/money",1000);`

**结果2:**`xxx/chatter/money`

- 私有名称

**格式:**以`~`开头的名称，适用于节点内部私有资源（如参数、调试话题）

**示例1:**

```
ros::NodeHandle nh("~");
ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",1000);
```

**结果1:**`/xxx/hello/chatter`

**示例2:**

```
ros::NodeHandle nh("~");
ros::Publisher pub = nh.advertise<std_msgs::String>("chatter/money",1000);
```

**结果2:**`/xxx/hello/chatter/money`

*PS:当使用*`~`*,而话题名称有时*`/`*开头时，那么话题名称是绝对的*

**示例3:**

```
ros::NodeHandle nh("~");
ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter/money",1000);
```

**结果3:**`/chatter/money`

注意，`Nodehandle` 传入的参数为：

| 传入参数         | 含义                         | 最终解析的命名空间                   |
| ---------------- | ---------------------------- | ------------------------------------ |
| `""`（空字符串） | **相对命名空间**（默认行为） | 相对于节点的命名空间（`__ns`）       |
| `"~"`            | **私有命名空间**             | 相对于**节点本身的名字**（`__name`） |
| `"/"`            | **全局命名空间**             | 根命名空间 `/`                       |
| `"/xxx"`         | **指定全局命名空间**         | `/xxx`                               |
| `"xxx"`          | **指定相对命名空间**         | `<node_namespace>/xxx`               |

---

## ROS参数名称设置

在ROS中节点名称话题名称可能出现重名的情况，同理参数名称也可能重名。

> 当参数名称重名时，那么就会产生覆盖，如何避免这种情况？

关于参数重名的处理，没有重映射实现，为了尽量的避免参数重名，都是使用为参数名添加前缀的方式，实现类似于话题名称，有全局、相对、和私有三种类型之分。

- 全局(参数名称直接参考ROS系统，与节点命名空间平级)
- 相对(参数名称参考的是节点的命名空间，与节点名称平级)
- 私有(参数名称参考节点名称，是节点名称的子级)

设置参数的方式也有三种:

- rosrun 命令
- launch 文件
- 编码实现

### rosrun设置参数

rosrun 在启动节点时，也可以设置参数:

**语法:** rosrun 包名 节点名称 _参数名:=参数值

- 设置参数

启动乌龟显示节点，并设置参数 A = 100

```bash
rosrun turtlesim turtlesim_node _A:=100
```

- 运行

`rosparam list`查看节点信息,显示结果:

```bash
/turtlesim/A
/turtlesim/background_b
/turtlesim/background_g
/turtlesim/background_r
```

结果显示，参数A前缀节点名称，也就是说rosrun执行设置参数参数名使用的是私有模式

### launch文件设置参数

通过 launch 文件设置参数的方式前面已经介绍过了，可以在 node 标签外，或 node 标签中通过 param 或  rosparam 来设置参数。在 node 标签外设置的参数是全局性质的，参考的是 / ，在 node 标签中设置的参数是私有性质的，参考的是  /命名空间/节点名称。

- 设置参数

以 param 标签为例，设置参数

```xml
<launch>

    <param name="p1" value="100" />
    <node pkg="turtlesim" type="turtlesim_node" name="t1">
        <param name="p2" value="100" />
    </node>

</launch>
```

- 运行

`rosparam list`查看节点信息,显示结果:

```
/p1
/t1/p1
```

### 编码设置参数

- ros::param设置参数

设置参数调用API是ros::param::set，该函数中，参数1传入参数名称，参数2是传入参数值，参数1中参数名称设置时，如果以 / 开头，那么就是全局参数，如果以 ~ 开头，那么就是私有参数，既不以 / 也不以 ~ 开头，那么就是相对参数。代码示例:

```cpp
ros::param::set("/set_A",100); //全局,和命名空间以及节点名称无关
ros::param::set("set_B",100); //相对,参考命名空间
ros::param::set("~set_C",100); //私有,参考命名空间与节点名称
```

运行时，假设设置的 namespace 为 xxx，节点名称为 yyy，使用 rosparam list 查看:

```
/set_A
/xxx/set_B
/xxx/yyy/set_C
```

- ros::NodeHandle设置参数

设置参数时，首先需要创建 NodeHandle 对象，然后调用该对象的 setParam  函数，该函数参数1为参数名，参数2为要设置的参数值，如果参数名以 / 开头，那么就是全局参数，如果参数名不以 /  开头，那么，该参数是相对参数还是私有参数与NodeHandle 对象有关，如果NodeHandle  对象创建时如果是调用的默认的无参构造，那么该参数是相对参数，如果NodeHandle 对象创建时是使用:

ros::NodeHandle nh("~")，那么该参数就是私有参数。代码示例:

```cpp
ros::NodeHandle nh;
nh.setParam("/nh_A",100); //全局,和命名空间以及节点名称无关

nh.setParam("nh_B",100); //相对,参考命名空间

ros::NodeHandle nh_private("~");
nh_private.setParam("nh_C",100);//私有,参考命名空间与节点名称
```

运行时，假设设置的 namespace 为 xxx，节点名称为 yyy，使用 rosparam list 查看:

```bash
/nh_A
/xxx/nh_B
/xxx/yyy/nh_C
```

## ROS分布式通信

ROS是一个分布式计算环境。一个运行中的ROS系统可以包含分布在多台计算机上多个节点。根据系统的配置方式，任何节点可能随时需要与任何其他节点进行通信。

因此，ROS对网络配置有某些要求：

- 所有端口上的所有机器之间必须有完整的双向连接。
- 每台计算机必须通过所有其他计算机都可以解析的名称来公告自己。

### 准备

先要保证不同计算机处于同一网络中，最好分别设置固定IP，如果为虚拟机，需要将网络适配器改为桥接模式；

### 配置文件修改

分别修改不同计算机的 `/etc/hosts`文件，在该文件中加入对方的IP地址和计算机名:

主机端:

```
从机的IP    从机计算机名
```

从机端:

```
主机的IP    主机计算机名
```

设置完毕，可以通过 ping 命令测试网络通信是否正常。

> IP地址查看名: `ifconfig`
>
> 计算机名称查看: `hostname`

### 配置主机IP

配置主机的 IP 地址

`~/.bashrc` 追加

```bash
export ROS_MASTER_URI=http://主机IP:11311
export ROS_HOSTNAME=主机IP
```

### 配置从机IP

配置从机的 IP 地址，从机可以有多台，每台都做如下设置:

`~/.bashrc` 追加

```bash
export ROS_MASTER_URI=http://主机IP:11311
export ROS_HOSTNAME=从机IP
```

### **测试**

1.主机启动 roscore(必须)

2.主机启动订阅节点，从机启动发布节点，测试通信是否正常

3.反向测试，主机启动发布节点，从机启动订阅节点，测试通信是否正常
