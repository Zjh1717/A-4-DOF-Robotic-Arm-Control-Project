# 交接文档

面向第一次接手本项目的开发者。配合 [README.md](README.md) 使用：README 讲“怎么跑”，
本文讲“怎么改、为什么这么设计、有什么坑”。

## 1. 项目现状

- 状态：ROS1 Noetic 实机闭环可运行（串口读取、KF 滤波、URDF 运动学、MPC 关节跟踪、
  RViz 可视化均已打通并真机验证）。
- 未实现：笛卡尔逆解/moveto、Gazebo/仿真、ros_control、力控、固件级急停。
- 真机验证记录见 README 的“Codex 实现记录 / 编译检查结果”。

## 2. 架构总览

```
ESP32/MicroPython 固件 (arm 对象, 舵机)
        | CH340 USB 串口, MicroPython REPL 文本协议
        v
arm_driver_node  (arm_driver)      —— 串口驱动 + code<->urdf 换算
        | /arm/joint_states_raw (URDF 弧度)
        v
state_estimator_node (arm_state_estimator) —— 每关节一阶卡尔曼滤波
        | /arm/joint_states (URDF 弧度, 含速度)
        v
mpc_controller_node (arm_controller) —— 关节空间 MPC
        | /arm/joint_command (Float64MultiArray[5], 前4弧度+夹爪原单位)
        v
arm_driver_node —— 转固件角度(度)下发 arm.bundle.move(...)

参考输入: /arm/end_effector_trajectory (JointTrajectory, 4关节, URDF弧度)
笛卡尔输入: /arm/target_pose (PoseStamped) —— 先解析 IK 再交给关节 MPC
可视化: robot_state_publisher 订阅 /arm/joint_states 发布 TF, RViz 显示
```

上位机入口脚本：`scripts/send_joint_goal.py`（发 4 关节目标）、
`scripts/send_pose_goal.py`（发末端 `xyz+四元数`，走 IK）。

## 3. 核心模块与职责

| 包 | 关键文件 | 职责 |
|---|---|---|
| arm_driver | serial_driver.cpp | termios 字节传输（协议无关，8N1 115200） |
| arm_driver | servo_protocol.h | 度↔弧度 + code↔urdf 换算（**唯一权威位置**） |
| arm_driver | servo_driver.cpp | REPL 驱动（等价 python_sdk.py 的 Arm 类），实现 ControlBase |
| arm_driver | arm_driver_node.cpp | 订阅 /arm/joint_command，发布 /arm/joint_states_raw |
| arm_state_estimator | kalman_filter.h/.cpp | 单关节 [q,dq] 卡尔曼滤波 |
| arm_state_estimator | state_estimator_node.cpp | raw → filtered，夹爪透传 |
| arm_controller | arm_kinematics.h/.cpp | 解析 URDF：FK、几何雅可比、限位（不硬编码 DH） |
| arm_controller | arm_ik.h/.cpp | 解析逆解（底座 yaw + 平面 3R，URDF 空间） |
| arm_controller | mpc_solver.h/.cpp | 投影梯度 MPC，状态[q,dq]、控制 q_dot、硬限位 |
| arm_controller | trajectory_handler.h/.cpp | 参考轨迹采样/看门狗 |
| arm_controller | mpc_controller_node.cpp | 控制主循环 |
| arm_description | urdf/arm_4dof.urdf | 唯一 URDF（rpy 已修正为 0 0 0） |
| arm_bringup | launch/arm_real.launch | 实机总启动 |

## 4. 关键设计决策

1. **REPL 文本协议而非二进制**：固件是 MicroPython REPL，命令形如
   `arm.bundle.move(ids=[...],vals=[...],speed=N)`，用 `>>>` 提示符做同步。
2. **关节约定换算集中化**：固件角度单位为度、零位竖直向上；URDF 为弧度、零位水平向前。
   三个平面关节（肩/肘/腕）方向取反，肩关节再叠加 -90°。全部在
   `servo_protocol.h` 的 `codeKinematicToUrdf/urdfKinematicToCode` 中完成，
   偏移量可经 `servo_config.yaml` 的 `joint2_offset_deg` 翻转。该换算已由
   `inv_geo.py` 的 `forward_3dof/4dof` 数值比对确认。
3. **MPC 不引求解器**：状态 [q,dq]、控制为关节速度 q_dot，代价为位置/速度/控制量
   二次型；用投影梯度 + 盒约束投影实现，关节限位在 rollout 中饱和处理（硬约束）。
4. **运动学直接读 URDF**：origin/axis 链式推 FK 与几何雅可比，连杆长度 0.06/0.04/0.025
   均来自 URDF，避免硬编码 DH 与 kdl_parser 依赖。
5. **点到点控制**：固件 `bundle.move` 阻塞执行完才回 `>>>`，驱动下发后等待提示符，
   故闭环为“读→解算→下发→读”。
6. **死区**：驱动层 `move_deadband_deg`（默认 0.5°）跳过与上次目标差小于阈值的指令，
   避免固件不响应的微小指令。
7. **笛卡尔接口走 IK**：`/arm/target_pose`（PoseStamped）先由 `arm_ik` 解析成 4 关节角，
   再复用关节 MPC 跟踪。只实现 yaw(绕Z)+pitch(绕Y) 两个姿态自由度，绕工具轴自转投影
   忽略；多解（肘上/肘下）选离当前构型最近者。
   `arm_ik` 与固件参考 `inv_geo.py` 的 `solution_arm` 几何等价（同为底座 yaw + 平面 3R
   余弦定理解），已交叉验证：`inv_geo` 的 IK 解经 code→urdf 换算后落回目标点，
   误差仅来自 `inv_geo` 对关节角 0.01°、FK 4 位小数的舍入（~2e-5 m）。

## 5. 配置说明

| 文件 | 关键项 | 含义 |
|---|---|---|
| arm_driver/config/servo_config.yaml | port/baudrate/read_rate_hz | 串口与读频率 |
| 同上 | move_speed | bundle.move 的 speed（未标定到 rad/s） |
| 同上 | move_deadband_deg | 下发死区（度） |
| 同上 | joint2_offset_deg | 肩关节零位偏移（默认 -90） |
| arm_state_estimator/config/kalman_params.yaml | dt/Q/R | 滤波周期与噪声（占位值） |
| arm_controller/config/mpc_params.yaml | horizon/dt/lr/max_iter | MPC 参数 |
| 同上 | trajectory_timeout_s | 轨迹有效窗口（到期仍跟踪最后路点） |
| arm_controller/config/kinematics_params.yaml | urdf_file | 兜底 URDF 路径（优先 /robot_description） |

## 6. 已知问题与待办

- `move_speed` 与 rad/s 映射未标定。
- KF 噪声 Q/R 为占位值，需真机标定。
- 固件对 <1° 指令不响应，收敛尾段停在距目标约 1~1.5°。
- `joint2_offset_deg` 默认 -90°，若真机方向相反需翻转（已参数化）。
- 固件基座高度与 URDF 肩高差 0.04m（仅影响笛卡尔坐标，不影响关节跟踪）。
- `emergencyStop()` 用 `arm.bundle.lock()` 尽力而为，无固件级急停。
- `src/arm_bringup/launch/arm__sim.launch` 为 0 字节占位（仿真未实现）。
- 关节限位紧：joint3 上界仅 45°，复位位姿已贴近该限位，邻近的笛卡尔目标可能被 IK
  判为不可达（属正确行为）。

## 7. 风险与常见坑

- **方向/零位**：任何真机调试前先确认 `joint2_offset_deg` 符号；改错会导致机械臂向
  相反方向运动。
- **串口独占**：驱动占用 `/dev/ttyUSB0`，勿同时开 Thonny/其它串口工具。
- **权限**：非 dialout 组用户打不开串口，见 README“串口权限”。
- **回显污染**：REPL 会回显命令，解析前必须剥离回显与 `>>>`（已在 servo_driver 处理）。
- **发布轨迹**：`rostopic pub` 的嵌套 YAML 易写错（`header.stamp: now` 不被支持），
  建议用 `scripts/send_joint_goal.py`。
- **构建产物误提交**：`build/`、`devel/` 历史上有被 git 跟踪，交付前需
  `git rm -r --cached build devel install`（见 README“交付前整理”）。

## 8. 发布 / 回滚

- 发布：`catkin_make` 全量通过即可；新机器从 `src/` 重建，不依赖 `build/devel`。
- 回滚：代码改动均可经 git 回退；固件与串口协议变更需同步回滚固件与
  `servo_protocol.h`，二者强耦合。
- 上线检查清单：串口权限、`joint2_offset_deg` 方向、真机“保持位姿”自检、限位回读。
