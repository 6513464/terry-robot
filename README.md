# DLO + DUFOMap：动态环境下的激光里程计与点云剔除系统

> 基于 ROS 2 / Gazebo 的 FishBot 仿真环境，集成 DLO 激光里程计，并使用 DUFOMap 探索动态点云检测与静态地图清理。

**环境：** Ubuntu 22.04 · ROS 2 Humble · Gazebo Classic · Python / C++
**关键词：** LiDAR Odometry · DLO · NanoGICP · Dynamic Point Removal · DUFOMap · PointCloud2 · TF · PCD

## 1. 项目简介


本项目以 FishBot 差速移动机器人为仿真载体，在带有移动汽车和行人的 Gazebo 场景中，研究动态物体对激光里程计和点云建图的影响。

机器人模型、ROS 2 基础工程及 Gazebo 仿真环境主要参考 **鱼香 ROS / FishBot 教学项目**进行学习和搭建；**本人重点完成的工作是 DLO 在现有仿真环境中的部署集成，以及面向 DUFOMap 的动态点云处理与数据适配流程。**

本仓库主要用于算法学习和工程验证，不宣称原创 DLO、DUFOMap 算法或 FishBot 机器人模型。

### 主要工作

* **DLO 接入：** 在已有 ROS 2 / Gazebo 环境中接入 `direct_lidar_odometry`，使用仿真三维激光点云开展里程计测试。
* **动态场景实验：** 利用场景中的 `dynamic_car` 和 `dynamic_person`，观察移动目标对点云及里程计的影响。
* **点云与位姿导出：** 编写 / 集成 ROS 2 采集节点，订阅 `/lidar_points`，结合 TF 将点云变换到统一坐标系，导出含 `VIEWPOINT` 的 PCD 文件。
* **对照数据准备：** 分别使用里程计相关 TF 和 Gazebo 仿真真值位姿导出数据，便于比较位姿来源对后续点云处理的影响。
* **DUFOMap 集成实验：** 将导出的点云组织为 DUFOMap 可读取的数据结构，探索基于历史空闲空间证据的动态点云剔除。

> **说明：** 当前上传的代码包含 DLO 启动调用、PCD 数据采集与仿真场景文件；**DLO 和 DUFOMap 的核心源码及完整 DUFOMap 运行配置不在本仓库中**。本项目的 DUFOMap 流程以数据导出和外部算法实验为主，不应理解为已经实现了“DUFOMap 在线过滤 → DLO 实时重定位”的闭环系统。

## 2. 系统流程

```text
Gazebo Classic / FishBot / Dynamic Objects
                    |
               /lidar_points
                    |
          +---------+--------------------+
          |                              |
          v                              v
          |                     Raw PointCloud2
          |                              |
     Odometry / TF                       |
          |                              |
          +--------------+---------------+
                         |
                         v
                collect_data.py
             TF: odom <- lidar_link
                         |
            PCD (odom frame + VIEWPOINT)
                         |
                         |
                         |
            DUFOMap (external project)
                         |
            Dynamic / Static Map Analysis


## 3. 项目目录

下面只列出与本项目主要实验相关的内容；工作空间中还包含导航和 ROS 2 教学练习包。

```text
terry_robot/                 # ROS 2 workspace（含 src、build、install、log）
└── src/
    ├── fishbot_description/
    │   ├── launch/
    │   │   ├── gazebo_sim.launch.py       # 启动 FishBot Gazebo 仿真
    │   │   └── collect_data.launch.py     # 联合启动仿真、外部 DLO 和采集节点
    │   ├── urdf/fishbot/plugins/
    │   │   └── gazebo_sensor_plugin.xacro # 仿真激光、IMU 等传感器
    │   └── world/
    │       └── custom_room.world          # 包含动态目标的场景
    ├── fishbot_application/
    │   └── fishbot_application/
    │       ├── move.py                    # 驱动仿真动态目标
    │       ├── collect_data.py            # 基于 TF 的点云 / 位姿导出
    │       └── collect_gazebo_data.py     # 基于 Gazebo GT 的点云导出
    ├── fishbot_navigation2/               # 扩展导航包，非本项目重点
```

## 4. 运行环境与依赖

项目源代码中的启动方式对应以下环境：

* Ubuntu 22.04
* ROS 2 Humble
* **Gazebo Classic**（使用 `gazebo_ros`、`spawn_entity.py`，不是 Gazebo Harmonic）
* `colcon`、`xacro`、`robot_state_publisher`、`gazebo_ros`、`gazebo_ros2_control`
* Python 包：`rclpy`、`sensor_msgs_py`、`gazebo_msgs`、`tf2_ros`、`PyKDL`
* 可选的外部项目：兼容 ROS 2 的 DLO 移植版本、DUFOMap

**重要：** [DLO 官方仓库](https://github.com/vectr-ucla/direct_lidar_odometry) 原始实现面向 ROS 1。本工程的 `collect_data.launch.py` 调用的是**另一个工作空间里已准备好的 ROS 2 版本**，不能直接把原版 ROS 1 仓库当作即插即用的 ROS 2 包。

## 5. 编译与启动

以下命令以当前终端位于**包含 `src/` 的工作空间根目录**为前提。压缩包保留原始目录层次时，该目录通常是 `terry_robot/terry_robot/`。

### 5.1 编译工作空间

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果出现缺少依赖的问题，请先根据缺失的 ROS 包安装对应依赖。项目中的部分 `package.xml` 仍是教学模板，未完整声明全部运行时依赖。

### 5.2 启动 Gazebo 仿真

```bash
ros2 launch fishbot_description gazebo_sim.launch.py
```

启动后检查三维点云话题：

```bash
ros2 topic list
ros2 topic info /lidar_points
```

### 5.3 启动 DLO

先在**另一个终端**加载你已配置好的 ROS 2 DLO 工作空间，然后运行：

```bash
# 请换成自己的 ROS 2 DLO 工作空间路径
source /path/to/dlo_ros2_ws/install/setup.bash
ros2 launch direct_lidar_odometry dlo.launch.py
```

检查 DLO 输入是否与 `/lidar_points` 匹配，并确认 TF 中存在后续导出所需的 `odom` 与 `lidar_link` 关系。输出话题与 TF 发布方式以实际使用的 DLO 移植版本为准。

### 5.4 启动动态目标

```bash
ros2 run fishbot_application move
```

该节点通过 `/gazebo/set_entity_state` 更新 `dynamic_car`、`dynamic_person` 的位置，模拟场景内的运动目标。

### 5.5 分别采集 DLO/TF 与 Gazebo GT 数据

```bash
# 终端 A：按 TF 位姿导出点云
ros2 run fishbot_application collect_data
```

输出位置：

```text
~/dufomap/data/my_data/pcd/000000.pcd
~/dufomap/data/my_data/pcd/000001.pcd
...
```

另开终端：

```bash
# 终端 B：按 Gazebo GT 位姿导出点云
ros2 run fishbot_application collect_gazebo_data
```

输出位置：

```text
~/dufomap/data/my_data_gazebo/pcd/000000.pcd
~/dufomap/data/my_data_gazebo/pcd/000001.pcd
...
```

PCD 包含 `x y z intensity` 字段，以及记录雷达位置与方向的 `VIEWPOINT`。代码中会将点云变换到相应的全局参考坐标系；因此**不能再将这些点当作未变换的雷达局部坐标**。

之后打开键盘控制节点控制fishbot进行移动循环采集仿真环境内的点：

```text
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```


### 5.6 使用现有的联合启动脚本（可选）

```bash
ros2 launch fishbot_description collect_data.launch.py
```

此脚本会按定时顺序启动仿真、外部 DLO、动态目标以及两套数据采集节点。

**运行前必须先修改：**

```text
src/fishbot_description/launch/collect_data.launch.py
```

脚本中的 `/home/tangrui/ros2_ws` 是原机器上的**绝对路径**，需要改成你自己的 DLO 工作空间。建议首次调试时先按上面步骤逐个启动，以便定位问题。

### 5.7 将数据交给 DUFOMap（外部工程）

本仓库负责将扫描导出到：

```text
~/dufomap/data/my_data/pcd/
~/dufomap/data/my_data_gazebo/pcd/
```

按 [DUFOMap 官方仓库](https://github.com/KTH-RPL/dufomap) 的说明，在**另行安装的 DUFOMap 项目目录**执行与数据格式对应的处理命令。其 Python 示例入口为：

```bash
python main.py --data_dir /path/to/data/my_data
```

其中 `--data_dir` 指向**包含 `pcd/` 子目录的父目录**，而不是某个单独 PCD 文件。该命令来自 DUFOMap 上游示例，压缩包本身未包含 `main.py`，需先完成上游依赖安装与数据检查。

### 5.8 导航节点的启动（可选）
在采集完数据进行动态点云删除之后可以选择启动导航节点：
有图导航启动：
```bash
ros2 launch fishbot_navigation2 navigation2.launch.py 

```


无图导航启动：
```bash
ros2 launch fishbot_navigation2_nomap navigation2.launch.py 

```
在无图导航中会自己显示时时运动的物体，控制小车移动的时候，小车会自动规划路径自动避让正在运动的人和车

## 6. 关键技术问题

### 6.1 为什么需要动态点剔除？

DLO 通过连续点云之间、以及当前点云与局部地图之间的配准估计运动。移动的行人、车辆可能引入不一致的几何对应关系，从而干扰配准或污染地图。

### 6.2 为什么尝试 DUFOMap？

DUFOMap 利用**被充分观测为空闲的空间**建立动态证据：如果某个位置曾被可靠地观测为空，其他时刻有点占据该位置，就可能对应动态结构。这与直接依靠物体类别或逐帧目标速度的方法不同。

### 6.3 里程计与动态检测是否存在循环依赖？

实际工程中会遇到：

```text
DLO 期望较干净的点云
          ↑
          |
动态剔除需要跨帧空间对齐
          |
          └── 通常需要位姿信息
```

本项目采用**先产生位姿/采集数据，再使用 DUFOMap 处理点云**的实验路径，并增加 Gazebo GT 位姿导出用于对照。

需要明确：**这不是完全无需位姿的实时在线动态剔除方案，也不等于已经解决了 DLO 与动态检测的闭环耦合问题。** 这是在里程计和位姿估计矛盾之间找到的新的动态点云去除的新思路

## 7. 已知限制与后续工作

* 当前仓库未打包 DLO 与 DUFOMap 的完整第三方源码、依赖和配置，因此**不能保证一条命令从零复现全部实验**。
* `collect_gazebo_data.py` 使用最新收到的 Gazebo 模型位姿，而不是严格按每帧雷达时间戳同步真值；高速运动下需进一步处理时序误差。


## 8. 参考项目与致谢

本项目建立在以下开源算法和教学资料之上，感谢原作者：

1. **FishBot / 鱼香 ROS**：机器人模型和 ROS 2 / Gazebo 基础仿真教学。
   [FishBot 官方代码](https://github.com/fishros/fishbot) · [教学网站](https://fishros.com/d2lros2/)
2. **Direct LiDAR Odometry (DLO)**：激光里程计方法。
   [官方代码](https://github.com/vectr-ucla/direct_lidar_odometry)
3. **DUFOMap: Efficient Dynamic Awareness Mapping**：基于历史空闲空间的动态感知和建图方法。
   [官方代码](https://github.com/KTH-RPL/dufomap)

**工作范围说明：** FishBot 基础仿真工程来自教学学习；DLO、DUFOMap 算法版权归其原作者。个人工作重点为**现有环境中的 DLO 部署、ROS 2 数据接口适配、PCD/位姿导出与 DUFOMap 动态点云处理实验**。仓库内第三方代码的使用应遵循各自许可证；整体仓库许可证需在梳理已有文件授权后确定。
