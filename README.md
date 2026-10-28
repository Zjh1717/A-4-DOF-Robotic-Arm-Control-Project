# 4-DOF 机械臂 ROS1 工程

ROS1 Noetic catkin 工作区：4-DOF 机械臂（底座 + 平面 3R）+ 夹爪。
下位机为 ESP32/MicroPython 固件，上位机通过 CH340 USB 串口以 MicroPython REPL
文本命令通信；控制器为基于 URDF 模型的关节空间 MPC。

## 使用方法

### 前置条件

- ROS1 Noetic（`/opt/ros/noetic`）。
- 已编译工作区（`run_arm.sh` 会在缺少 `devel/` 时自动 `catkin_make`）。
- CH340 USB 驱动（Ubuntu 一般已内置），插上后出现 `/dev/ttyUSB0`。

### 串口权限（一次性）

当前用户必须在 `dialout` 组，否则无法打开 `/dev/ttyUSB0`：

```
sudo usermod -aG dialout $USER
```

注销并重新登录后生效；临时方案（重新插拔后失效）：

```
sudo chmod 666 /dev/ttyUSB0
```

### 一键启动

```
cd <工作区路径>
./run_arm.sh
```

可选环境变量：

```
ARM_PORT=/dev/ttyUSB1 ./run_arm.sh   # 指定串口
RUN_ARM_BUILD=1 ./run_arm.sh         # 强制重新编译
```

### 验证数据流（另开终端）

```
source /opt/ros/noetic/setup.bash
source <工作区路径>/devel/setup.bash
rostopic echo /arm/joint_states_raw    # 驱动读回的原始关节角（URDF 弧度）
rostopic echo /arm/joint_states        # KF 滤波后
rostopic echo /arm/joint_command       # MPC 下发的关节目标
```

### 发送关节目标（先保持，再小幅运动）

先读取当前 4 个运动学关节角（rad），用“当前值”做保持测试，确认换算回环不会引起
运动；确认方向无误后再小幅改某一关节：

```
# 发送关节目标（URDF 空间，弧度）
python3 scripts/send_joint_goal.py 0.0 -1.5708 0.7854 0.7941
```

也可用 `rostopic pub`，但嵌套消息的 YAML 写法易错（`header.stamp: now` 不被支持），
推荐用上面的脚本。

夹爪为 id4，按固件原单位（0~30）透传，MPC 不控制夹爪。

### 零位 / 方向校核

- 固件零位=竖直向上，URDF 零位=水平向前，二者在 joint2 相差 90°；肩/肘/腕方向取反。
- 换算集中在 `src/arm_driver/include/arm_driver/servo_protocol.h`。
- 偏移量由 `src/arm_driver/config/servo_config.yaml` 的 `joint2_offset_deg` 控制（默认
  `-90.0`）。若真机姿态相反，把它改为 `90.0` 后重启驱动即可。

### 安全

- 下位机 `bundle.move` 为阻塞执行，控制器为点到点闭环。
- 当前固件无专用急停指令，`emergencyStop()` 以 `arm.bundle.lock()` 失能作为尽力而为。
- 紧急情况请直接断开串口/断电。

### 关键参数

- `src/arm_driver/config/servo_config.yaml` 的 `move_deadband_deg`（默认 0.5°）：
  新目标与上次已下发目标的差值小于该值时不重复下发，避免固件不响应的微小指令。
- `src/arm_controller/config/mpc_params.yaml` 的 `trajectory_timeout_s`（默认 2.0 s）：
  参考轨迹有效窗口；到期后控制器仍跟踪最后一个路点以保证收敛。

## Codex 实现记录

### 修改/新增文件

修改：

- `src/arm_driver/include/arm_driver/serial_driver.h` — 串口字节传输层接口。
- `src/arm_driver/include/arm_driver/servo_driver.h` — REPL 驱动（等价 `Arm` 类）接口。
- `src/arm_driver/include/arm_driver/servo_protocol.h` — 度↔弧度 + code↔urdf 集中转换。
- `src/arm_driver/src/serial_driver.cpp` — termios 串口打开/读写/超时/flush。
- `src/arm_driver/src/servo_driver.cpp` — REPL 命令、读回解析、关节命令/状态转换。
- `src/arm_driver/src/arm_driver_node.cpp` — 订阅 `/arm/joint_command`，发布 `/arm/joint_states_raw`。
- `src/arm_state_estimator/src/state_estimator_node.cpp` — 订阅 `_raw`、KF 滤波、发布 `/arm/joint_states`。
- `src/arm_controller/include/arm_controller/arm_kinematics.h` — URDF 解析/FK/雅可比/限位接口。
- `src/arm_controller/src/arm_kinematics.cpp` — URDF 解析、FK、几何雅可比实现。
- `src/arm_controller/include/arm_controller/mpc_solver.h` — MPC 求解器接口（控制量=速度）。
- `src/arm_controller/src/mpc_solver.cpp` — 投影梯度 MPC（状态 [q,dq]，硬限位）。
- `src/arm_controller/include/arm_controller/trajectory_handler.h` — 参考轨迹插值/看门狗接口。
- `src/arm_controller/src/trajectory_handler.cpp` — 参考轨迹采样实现。
- `src/arm_controller/src/mpc_controller_node.cpp` — 控制器节点（URDF 空间，弧度）。
- `src/arm_controller/config/mpc_params.yaml` — MPC 参数（限位改由 URDF 提供）。
- `src/arm_bringup/launch/arm_real.launch` — 修正 URDF 路径、话题 remap、加载节点。
- `src/arm_description/CMakeLists.txt` — 只安装实际存在的 `urdf`/`rviz` 目录。

新增：

- `src/arm_description/urdf/arm_4dof.urdf` — 本工程唯一使用的 URDF（joint2 `rpy` 已修正为 `0 0 0`）。
- `src/arm_description/rviz/arm_display.rviz` — 基础 RViz 配置（RobotModel + TF）。
- `src/arm_driver/config/servo_config.yaml` — 串口/REPL 驱动参数。
- `src/arm_state_estimator/config/kalman_params.yaml` — 估计器参数。
- `src/arm_state_estimator/launch/estimator.launch` — 估计器启动文件。
- `src/arm_controller/config/kinematics_params.yaml` — 运动学来源（兜底 urdf_file）。
- `run_arm.sh` — 根目录真机一键启动脚本（可选 `ARM_PORT`、`RUN_ARM_BUILD=1`）。
- `scripts/send_joint_goal.py` — 发送单个 4 关节目标（URDF 弧度）给 MPC。

保留未改：`arm_driver/msg/ArmJointState.msg`、`control_base.h`、`kalman_filter.h/.cpp`、
各包 `CMakeLists.txt`/`package.xml`（driver 的 CMake 已能编译，无需改动）。

### 未完成项

- `bundle.move` 的 `speed` 与 rad/s 的映射未标定，见 `servo_config.yaml` 的 TODO。
- 下位机 `bundle.move` 为阻塞执行，控制闭环为点到点（读→解算→下发→再读），速率受动作耗时限制。
- 夹爪按固件原单位透传，未做运动学、限位或标定。
- joint2 零位偏移默认 -90°，需真机校核；翻转 `joint2_offset_deg` 即可。
- KF 过程/测量噪声 Q/R 为占位值，需真机标定。
- MPC 投影梯度步长/迭代次数需调参；未接入外部 QP 求解器。
- `emergencyStop()` 用 `arm.bundle.lock()` 尽力而为，固件无专用急停指令。
- 未实现：笛卡尔 moveto/逆解、Gazebo 仿真、ros_control、力控。

### 假设

- 固件关节角单位为度；REPL 命令集以 `python_sdk.py` 的 `Arm` 类为准。
- 4 个运动学关节（底座/肩/肘/腕）+ 1 个夹爪（id4，原单位透传）。
- 控制量 = 关节速度（q_dot），MPC 状态 [q,dq]，关节限位为硬约束（取 URDF `lower/upper/velocity`）。
- 速度由状态估计器 KF 估计，驱动只发布位置。
- URDF 为唯一运动学来源，连杆长度解析自 origin/axis（0.06/0.04/0.025），不硬编码 DH。
- 平面三关节（肩/肘/腕）与固件方向相反需取反号，肩关节再叠加 -90° 零位偏移（已由 `inv_geo.py` 正解数值比对确认）。

### 节点频率与通信接口

| 节点 | 频率 | 订阅 | 发布 | 说明 |
| --- | --- | --- | --- | --- |
| `arm_driver_node` | 50 Hz | `/arm/joint_command` (`Float64MultiArray[5]`) | `/arm/joint_states_raw` (`JointState`) | 前 4 元素弧度(URDF)，第 5 夹爪原单位 |
| `state_estimator_node` | 50 Hz | `/arm/joint_states_raw` | `/arm/joint_states` | KF 滤波，夹爪透传 |
| `mpc_controller_node` | 20 Hz | `/arm/joint_states`、`/arm/end_effector_trajectory` | `/arm/joint_command` | 关节空间 MPC，URDF 弧度 |
| `robot_state_publisher` | 事件 | `/arm/joint_states`（remap） | TF | 由 `arm_real.launch` 启动 |

### 编译/检查结果

- `catkin_make` 全量通过：`arm_driver_node`、`state_estimator_node`、`mpc_controller_node` 及 3 个库均成功生成。
- 离线 FK 校验通过：`numJoints=4`，URDF 限位/速度解析正确，已知位姿 FK 与 `inv_geo.py` 正解一致。
- ROS 冒烟测试通过：`mpc_controller_node` 启动、从 `/robot_description` 加载 URDF、按 20 Hz 发布 5 元素 `/arm/joint_command`。
- 模拟下位机（pty 伪串口）闭环自测通过：驱动读回 `[10,20,30,40,5]°` 正确转换为
  URDF 弧度，估计器滤波输出正确，控制器按 URDF 限位钳位并下发，驱动反向换算回代码度
  与输入一致（见 `README` 使用方法）。
- 真机只读联调通过：REPL 握手成功，读回代码度 `[0,0,-45,-45.5,0]`，正确转换为
  URDF 弧度 `[0,-1.5708,0.7854,0.7941,0]`。
- 真机“保持位姿”闭环通过：控制器下发目标 ≈ 当前位姿（误差 ~1e-10 rad），机械臂不动。
- 真机小幅运动验证通过：底座 joint1 指令 +0.1 rad，实测转至 ~+0.0785 rad（方向/量级正确），
  回零后 ~+0.0262 rad。收敛尾段受固件死区限制（<1° 的微小指令不被舵机执行），
  属固件/硬件特性，软件侧已正确输出目标。
