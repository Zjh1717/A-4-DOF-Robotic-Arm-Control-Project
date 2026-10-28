# 4-DOF 机械臂 ROS1 工程

ROS1 Noetic catkin 工作区，控制一台 4-DOF 机械臂（底座旋转 + 平面 3R 俯仰臂）+ 夹爪。
下位机为 ESP32/MicroPython 固件，上位机通过 CH340 USB 串口以 MicroPython REPL
文本命令通信；控制器为基于 URDF 模型的关节空间 MPC。

## 功能

- 串口驱动：等价 `python_sdk.py` 中 `Arm` 类的 C++ REPL 驱动（连接、读关节、点动、失能）。
- 状态估计：每关节一阶卡尔曼滤波，由位置观测估计角度/角速度。
- 运动学：从 URDF 解析 4-DOF 正解与几何雅可比、关节限位（不硬编码 DH、不依赖 kdl）。
- 笛卡尔逆解：解析 IK（底座 yaw + 平面 3R），把末端 `(x,y,z)+四元数` 目标解成关节角。
- 控制：关节空间 MPC（状态 [q,dq]、控制 q_dot、关节限位硬约束，纯 Eigen 无求解器依赖）。
- 可视化：`robot_state_publisher` + RViz 基础配置。

## 技术栈

- ROS1 Noetic（Ubuntu 20.04）、catkin_make、C++14、Eigen3。
- 串口：termios（协议无关字节层）+ MicroPython REPL 文本协议。
- 消息：`sensor_msgs/JointState`、`trajectory_msgs/JointTrajectory`、
  `std_msgs/Float64MultiArray`，以及本包自定义 `arm_driver/ArmJointState`。

## 目录结构

```
.
├── run_arm.sh                 # 一键启动脚本
├── scripts/
│   ├── send_joint_goal.py     # 发送 4 关节目标（URDF 弧度）
│   └── send_pose_goal.py      # 发送末端位姿（xyz + 四元数）
├── src/
│   ├── arm_description/       # URDF + RViz（无代码）
│   │   ├── urdf/arm_4dof.urdf
│   │   └── rviz/arm_display.rviz
│   ├── arm_driver/            # 串口驱动 + 单位/零位换算
│   │   ├── include/arm_driver/
│   │   ├── src/
│   │   ├── msg/ArmJointState.msg
│   │   └── config/servo_config.yaml
│   ├── arm_state_estimator/   # 卡尔曼滤波
│   │   ├── include/arm_state_estimator/
│   │   ├── src/
│   │   ├── config/kalman_params.yaml
│   │   └── launch/estimator.launch
│   ├── arm_controller/        # URDF 运动学/IK + MPC + 轨迹处理
│   │   ├── include/arm_controller/
│   │   ├── src/
│   │   └── config/{mpc_params,kinematics_params}.yaml
│   └── arm_bringup/           # 实机总启动
│       └── launch/arm_real.launch
├── HANDOVER.md                # 交接/架构文档
└── .gitignore
```

`build/`、`devel/`、`install/` 为 catkin 构建产物，已由 `.gitignore` 忽略。

## 环境依赖

- Ubuntu 20.04 + ROS1 Noetic（`ros-noetic-desktop-full`）。
- 系统库：`libeigen3-dev`（Eigen3）、`libserial-dev`（serial）。
- ROS 包：`roscpp`、`rospy`、`std_msgs`、`sensor_msgs`、`trajectory_msgs`、
  `geometry_msgs`、`robot_state_publisher`、`rviz`、`joint_state_publisher`、
  `urdf`、`xacro`。
- 硬件：CH340 USB 串口（Ubuntu 内置驱动），ESP32/MicroPython 下位机。

## 安装

```bash
cd <工作区路径>          # 即本仓库根目录
source /opt/ros/noetic/setup.bash
catkin_make              # 首次构建；run_arm.sh 也会在缺少 devel/ 时自动构建
source devel/setup.bash
```

串口权限（一次性，重新登录后生效）：

```bash
sudo usermod -aG dialout $USER
```

临时方案（重新插拔后失效）：`sudo chmod 666 /dev/ttyUSB0`。

## 配置

所有参数集中在各包 `config/*.yaml`，由 `arm_real.launch` 加载：

| 文件 | 关键项 | 说明 |
|---|---|---|
| `arm_driver/config/servo_config.yaml` | `port`、`baudrate`、`read_rate_hz` | 串口与读频率 |
| 同上 | `move_speed`、`move_deadband_deg` | 点动速度、下发死区（度） |
| 同上 | `joint2_offset_deg` | 肩关节零位偏移（默认 -90） |
| `arm_state_estimator/config/kalman_params.yaml` | `dt`、过程/测量噪声 | 滤波参数（占位值） |
| `arm_controller/config/mpc_params.yaml` | `horizon`、`dt`、`lr`、`max_iter`、`trajectory_timeout_s` | MPC 参数 |
| `arm_controller/config/kinematics_params.yaml` | `urdf_file` | 兜底 URDF 路径（优先 `/robot_description`） |

## 运行

```bash
./run_arm.sh                          # 启动驱动+估计器+MPC+RViz
ARM_PORT=/dev/ttyUSB1 ./run_arm.sh    # 指定串口
RUN_ARM_BUILD=1 ./run_arm.sh          # 强制重新编译
```

验证数据流（另开终端，先 `source devel/setup.bash`）：

```bash
rostopic echo /arm/joint_states_raw    # 驱动读回的原始关节角（URDF 弧度）
rostopic echo /arm/joint_states        # KF 滤波后
rostopic echo /arm/joint_command       # MPC 下发的关节目标
```

发送关节目标（先保持当前位姿验证方向，再小幅运动）：

```bash
python3 scripts/send_joint_goal.py 0.0 -1.5708 0.7854 0.7941
```

发送末端位姿目标（笛卡尔，`base_link` 系，位置米 + 四元数 `x y z w`）：

```bash
python3 scripts/send_pose_goal.py <x> <y> <z> <qx> <qy> <qz> <qw>
```

控制器收到 `/arm/target_pose` 后先做 IK 再交给关节 MPC 跟踪；不可达/超限时保持
当前位姿并告警。注意本臂只有 4 个关节，姿态仅能实现“绕 Z 的 yaw + 绕 Y 的 pitch”，
绕工具轴的自转会被投影忽略。

夹爪为 id4，按固件原单位（0~30）透传，MPC 不控制夹爪。

## 工作空间与位姿有效范围

对全关节空间（URDF 限位）数值采样 FK 得到，坐标均为 `base_link` 系：

- 水平半径（到 Z 轴距离）`r`：0 ~ 0.180 m（机械臂水平伸直时最大）。
- 高度 `z`：-0.097 ~ 0.245 m（肩关节位于 z=0.065 m）。
- 工具俯仰角 `φ`（绕 Y）：可覆盖 -180° ~ +180°。
- 空间形态：绕底座 Z 轴的回转体；底座 yaw 限 ±90°，但平面臂可向后弯，整体近似全周，
  **正向最远 0.180 m、反向最远约 0.125 m**（关节限位不对称）。

按高度切片（该高度下可达的水平半径 r）：

| z (m) | r 可达范围 (m) |
| --- | --- |
| -0.09 | 0.007 ~ 0.096 |
| -0.05 | 0 ~ 0.142 |
| 0.00 | 0.079 ~ 0.170 |
| +0.05 | 0.025 ~ 0.180 |
| +0.10 | 0 ~ 0.177 |
| +0.15 | 0 ~ 0.161 |
| +0.20 | 0 ~ 0.124 |
| +0.24 | 0 ~ 0.059 |

注意：`x=y=0`（正上方/正下方）为底座奇异点，IK 会保持当前 yaw；某点是否可达最终由
IK 精确判断，超出工作空间或超限时控制器保持当前位姿并告警。

## 测试

- 构建后冒烟：启动 `./run_arm.sh`，依次检查三个话题是否有数据、`/arm/joint_command`
  是否约等于 `/arm/joint_states`。
- 无硬件自测：可用 pty 伪串口模拟下位机（开发调试用，仓库不含该脚本）。
- 真机“保持位姿”自检：发送等于当前值的关节目标，机械臂应不动。
- 离线 FK 校验：`arm_kinematics.cpp` 解析 URDF 后 `numJoints()==4`，限位/速度正确。
- 离线 IK 校验：`arm_ik.cpp` 与 URDF FK 回环（位置/俯仰误差 ~1e-16），并已与
  `inv_geo.py` 的 `solution_arm` 交叉验证（落点误差 ~2e-5 m，来自其舍入）。

## 构建

```bash
source /opt/ros/noetic/setup.bash
catkin_make
```

产物：`devel/lib/arm_driver/arm_driver_node`、
`devel/lib/arm_state_estimator/state_estimator_node`、
`devel/lib/arm_controller/mpc_controller_node`，以及三个 `.so` 库。

## 部署 / 交付前整理

历史提交中 `build/`、`devel/` 曾被视为普通文件跟踪。上传前执行（只改 git 索引，
不删磁盘文件，功能不受影响）：

```bash
git rm -r --cached build devel install   # install 目录若存在
git add .gitignore
git commit -m "chore: stop tracking catkin build artifacts"
```

> 若需彻底缩小 `.git` 体积（历史已含约 48MB 构建产物），需用 `git filter-repo`
> 重写历史；有风险，默认不做。

## 常见问题

- **打不开 `/dev/ttyUSB0`**：当前用户不在 `dialout` 组，见“安装”。
- **机械臂方向相反**：把 `servo_config.yaml` 的 `joint2_offset_deg` 翻转为 `90.0` 重启。
- **轨迹不生效 / `rostopic pub` 报 “No field name”**：嵌套 YAML 写法易错
  （`header.stamp: now` 不被支持），改用 `scripts/send_joint_goal.py`。
- **收敛停在目标附近 1~1.5°**：固件对 <1° 指令不响应（死区），属硬件特性；
  软件已正确输出目标，可通过调 `move_deadband_deg` 控制下发粒度。
- **串口被占用**：Thonny 或其它串口工具与驱动互斥，关闭后再启动。
- **IK 报“不可达”**：多为目标超出工作空间或超出 URDF 关节限位（尤其 joint3 上界
  仅 45°）。属正确行为，控制器会保持当前位姿；换一个更近/更低的目标重试。

## 开发指南

- 分层：感知（driver/estimator）→ 规划（trajectory_handler）→ 控制（MPC）→
  执行（driver）。层间仅通过话题/消息通信，节点内不直接调用其它包。
- 单位与坐标约定：控制器内部一律 URDF 空间（弧度）；固件角度（度）与零位换算只在
  `servo_protocol.h` 中集中处理，其它代码不得自行换算。
- 新增话题/服务先在对应包声明消息或复用 `std_msgs`/`sensor_msgs`/`trajectory_msgs`。
- 公共 API 注释与复杂逻辑说明见各头文件；更完整的设计决策见 [HANDOVER.md](HANDOVER.md)。

## 节点频率与通信接口

| 节点 | 频率 | 订阅 | 发布 | 说明 |
| --- | --- | --- | --- | --- |
| `arm_driver_node` | 50 Hz | `/arm/joint_command` (`Float64MultiArray[5]`) | `/arm/joint_states_raw` (`JointState`) | 前 4 元素弧度(URDF)，第 5 夹爪原单位 |
| `state_estimator_node` | 50 Hz | `/arm/joint_states_raw` | `/arm/joint_states` | KF 滤波，夹爪透传 |
| `mpc_controller_node` | 20 Hz | `/arm/joint_states`、`/arm/end_effector_trajectory`、`/arm/target_pose` | `/arm/joint_command` | 关节空间 MPC（含 IK），URDF 弧度 |
| `robot_state_publisher` | 事件 | `/arm/joint_states`（remap） | TF | 由 `arm_real.launch` 启动 |

## Codex 实现记录

### 修改/新增文件

修改：`serial_driver.h/.cpp`、`servo_driver.h/.cpp`、`servo_protocol.h`、
`arm_driver_node.cpp`、`state_estimator_node.cpp`、`arm_kinematics.h/.cpp`、
`mpc_solver.h/.cpp`、`trajectory_handler.h/.cpp`、`mpc_controller_node.cpp`、
`arm_controller/package.xml`、`arm_controller/CMakeLists.txt`、`mpc_params.yaml`、
`arm_real.launch`、`arm_description/CMakeLists.txt`。

新增：`src/arm_description/urdf/arm_4dof.urdf`（joint2 `rpy=0 0 0`）、
`src/arm_description/rviz/arm_display.rviz`、`src/arm_driver/config/servo_config.yaml`、
`src/arm_state_estimator/config/kalman_params.yaml`、`launch/estimator.launch`、
`src/arm_controller/config/kinematics_params.yaml`、`run_arm.sh`、
`src/arm_controller/include/arm_controller/arm_ik.h`、
`src/arm_controller/src/arm_ik.cpp`、`scripts/send_joint_goal.py`、
`scripts/send_pose_goal.py`、`.gitignore`、`HANDOVER.md`。

### 未完成项

- `bundle.move` 的 `speed` 与 rad/s 映射未标定。
- 下位机 `bundle.move` 阻塞执行，控制闭环为点到点。
- 夹爪按原单位透传，未做运动学/限位。
- `joint2_offset_deg` 默认 -90°，需真机最终校核。
- KF 噪声 Q/R 为占位值；MPC 步长/迭代需调参。
- `emergencyStop()` 以 `arm.bundle.lock()` 尽力而为。
- 未实现：固件 `moveto` 级笛卡尔直达、Gazebo 仿真、ros_control、力控。

### 关键假设

- 固件关节角单位为度；REPL 命令集以 `python_sdk.py` 的 `Arm` 类为准。
- 4 运动学关节 + 1 夹爪；夹爪 id4 原单位透传。
- 控制量 = 关节速度 q_dot，状态 [q,dq]，限位取 URDF `lower/upper/velocity`。
- 速度由 KF 估计，驱动只发布位置。
- URDF 为唯一运动学来源，连杆长度解析自 origin/axis，不硬编码 DH。
- 平面三关节方向取反 + 肩关节 -90° 零位偏移（已由 `inv_geo.py` 正解数值比对确认）。

### 编译/检查结果

- `catkin_make` 全量通过（三个节点 + 三个库）。
- 离线 FK 校验通过（`numJoints=4`，限位/速度解析正确，FK 与 `inv_geo.py` 一致）。
- pty 伪串口闭环自测通过（换算、滤波、限位钳位、反向下发一致）。
- 真机只读联调、保持位姿、小幅运动均验证通过（详见“功能”与 HANDOVER 的风险说明）。
