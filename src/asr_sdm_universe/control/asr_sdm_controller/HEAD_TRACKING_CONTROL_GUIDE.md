# 头部跟踪控制算法完整指南

本文档提供头部跟踪（Front-Unit-Following）控制算法的完整使用、验证和故障排除指南。

## 目录

1. [概述](#概述)
2. [快速开始](#快速开始)
3. [详细运行步骤](#详细运行步骤)
4. [RViz可视化验证](#rviz可视化验证)
5. [验证检查清单](#验证检查清单)
6. [验证结果](#验证结果)
7. [故障排除](#故障排除)
8. [常见问题](#常见问题)
9. [性能指标](#性能指标)

---

## 概述

### 控制算法原理

头部跟踪控制算法实现了论文中的 front-unit-following 控制方法，使蛇形机器人的后续关节能够跟随前单元（头部）的运动轨迹。

**核心公式**：
- 第一个关节：`φ̇₁ = -(2/L)·v₁·sin(φ₁) - ω₁·(2·cos(φ₁)+1)`
- 后续关节（i>1）：`φ̇ᵢ = -(2/L)·v̄ᵢ·sin(φᵢ) - (ω₁+Σφ̇ⱼ)(cos(φᵢ)+1)`

其中：
- `L` = 段长度（默认 0.18 m）
- `v₁` = 前单元线速度
- `ω₁` = 前单元角速度
- `φᵢ` = 第 i 个关节角度

### 系统架构

```
前单元速度命令 (Twist)
    ↓
asr_sdm_controller 节点
    ↓
控制命令 (ControlCmd)
    ↓
关节状态更新 (JointState)
    ↓
RViz 可视化
```

---

## 快速开始

### 方法 1: 使用验证脚本（推荐）

最简单的验证方法，自动运行所有测试场景：

```bash
cd /home/wwlwwl/asr_sdm_ws
source install/setup.bash

# 终端 1: 启动控制器
ros2 run asr_sdm_controller asr_sdm_controller

# 终端 2: 启动 robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro src/asr_sdm_universe/robot/asr_sdm_description/urdf/underwater_snakerobot.urdf.xacro)"

# 终端 3: 启动 RViz
rviz2

# 终端 4: 运行验证脚本
python3 src/asr_sdm_universe/control/asr_sdm_controller/verify_head_tracking_rviz.py
```

脚本会自动运行 45 秒，包含 4 个测试场景。

### 方法 2: 手动验证

逐步验证每个功能：

```bash
# 终端 1: 启动控制器
ros2 run asr_sdm_controller asr_sdm_controller

# 终端 2: 发布关节状态
ros2 topic pub /joint_states sensor_msgs/msg/JointState "
header: {frame_id: 'base_link'}
name: ['base_link_to_link_2','link_2_to_link_3','link_3_to_link_4','link_4_to_link_5']
position: [0.0, 0.0, 0.0, 0.0]
velocity: [0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0, 0.0]
" --rate 10

# 终端 3: 发布速度命令
ros2 topic pub /asr_sdm_controller/input/front_unit_velocity geometry_msgs/msg/Twist "
{linear: {x: 0.1}, angular: {z: 0.2}}
" --rate 1

# 终端 4: 查看控制命令
ros2 topic echo /asr_sdm_controller/output/control_cmd
```

---

## 详细运行步骤

### 一、准备工作

1. **构建工作空间**：
```bash
cd /home/wwlwwl/asr_sdm_ws
source install/setup.bash
colcon build --packages-select asr_sdm_controller asr_sdm_description --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

2. **确认关键文件存在**：
   - 控制器节点：`src/asr_sdm_universe/control/asr_sdm_controller/src/asr_sdm_controller_node.cpp`
   - 验证脚本：`src/asr_sdm_universe/control/asr_sdm_controller/verify_head_tracking_rviz.py`
   - URDF 模型：`src/asr_sdm_universe/robot/asr_sdm_description/urdf/underwater_snakerobot.urdf.xacro`

### 二、启动步骤（按顺序）

#### 步骤 1：清理旧进程
```bash
pkill -f robot_state_publisher
pkill -f joint_state_publisher
pkill -f joint_state_publisher_gui
pkill -f asr_sdm_controller
pkill -f verify_head_tracking_rviz.py
pkill -f rviz2
```

#### 步骤 2：启动 robot_state_publisher（终端 1）
```bash
cd /home/wwlwwl/asr_sdm_ws
source install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro src/asr_sdm_universe/robot/asr_sdm_description/urdf/underwater_snakerobot.urdf.xacro)"
```
**保持运行，不要关闭**

#### 步骤 3：启动控制器节点（终端 2）
```bash
cd /home/wwlwwl/asr_sdm_ws
source install/setup.bash
ros2 run asr_sdm_controller asr_sdm_controller
```

**默认参数**：
- `segment_length=0.18` m
- `num_segments=4`
- `control_frequency=10.0` Hz

**保持运行，不要关闭**

#### 步骤 4：启动 RViz（终端 3）
```bash
cd /home/wwlwwl/asr_sdm_ws
source install/setup.bash
rviz2
```

**RViz 配置**：
1. **Fixed Frame**：设为 `odom`（避免 world->odom 报错）
2. **添加 RobotModel**：
   - Topic: `/robot_description`
   - 如需隐藏螺旋桨：RobotModel → Links → 取消勾选 `screw_*`
3. **添加 Marker（显示轨迹）**：
   - Topic: `/head_path`
   - Type: Line Strip
   - 颜色：绿色（已在脚本中设置）

#### 步骤 5：运行验证脚本（终端 4）
```bash
cd /home/wwlwwl/asr_sdm_ws
source install/setup.bash
python3 src/asr_sdm_universe/control/asr_sdm_controller/verify_head_tracking_rviz.py
```

脚本会自动运行 45 秒，包含以下场景：

- **0–15 秒**：圆形轨迹（v1=0.075 m/s, ω1=0.15 rad/s）
- **15–25 秒**：直线运动（v1=0.12 m/s, ω1=0）
- **25–40 秒**：S 形轨迹（v1=0.1 m/s, ω1=0.12×sin(0.3t)）
- **40 秒后**：停止（v1=0, ω1=0，关节角度衰减）

### 三、验证检查点

1. **检查控制器心跳**：
```bash
ros2 topic echo /asr_sdm_controller/output/controller/heartbeat
```
应每 1.5 秒看到 "Control Node Heartbeat"

2. **检查控制命令输出**：
```bash
ros2 topic echo /asr_sdm_controller/output/control_cmd
```
应看到各单元的 `UnitCmd` 消息，包含关节角度目标

3. **检查关节状态**：
```bash
ros2 topic echo /joint_states
```
应看到 4 个主体关节 + 10 个螺旋桨关节的状态

4. **检查 TF 树**：
```bash
ros2 run tf2_tools view_frames
```
应看到完整的 TF 树：`odom -> base_link -> link_2 -> ... -> link_5`

---

## RViz可视化验证

### 预期效果

在 RViz 中应看到：

1. **轨迹形状**：
   - 0–15 秒：圆弧路径（半径约 0.5 m）
   - 15–25 秒：直线路径
   - 25–40 秒：S 形路径
   - 40 秒后：停止

2. **身体形态**：
   - 水平面内左右弯曲（关节绕 z 轴）
   - 圆弧阶段：身体形成弯曲队形
   - 直线阶段：逐步拉直
   - S 形阶段：左右小幅摆动
   - 停止后：曲率衰减

3. **轨迹可视化**：
   - `/head_path` 显示绿色路径线
   - 路径随时间绘制，形成圆弧/直线/S 曲线

### 验证场景详解

#### 场景 1: 圆形轨迹跟踪 (0-15秒)

**输入命令**：
- `v1 = 0.075 m/s` (线速度)
- `omega1 = 0.15 rad/s` (角速度)

**预期效果**：
- 机器人身体逐渐弯曲成弧形
- 第一个关节角度变为负值（根据公式：φ̇₁ = -ω₁ * (2*cos(0) + 1) = -0.45 rad/s）
- 后续关节跟随前一个关节，形成平滑的弧形

**验证点**：
- ✓ 身体是否呈现弧形？
- ✓ 关节角度是否平滑变化？
- ✓ 是否跟随圆形轨迹？

#### 场景 2: 直线运动 (15-25秒)

**输入命令**：
- `v1 = 0.12 m/s`
- `omega1 = 0.0 rad/s`

**预期效果**：
- 机器人身体逐渐变直
- 关节角度逐渐回到接近0
- 机器人保持直线运动

**验证点**：
- ✓ 身体是否变直？
- ✓ 关节角度是否接近0？
- ✓ 是否保持直线运动？

#### 场景 3: S形轨迹 (25-40秒)

**输入命令**：
- `v1 = 0.1 m/s`
- `omega1 = 0.12 * sin(0.3*t) rad/s` (正弦变化)

**预期效果**：
- 机器人身体左右摆动
- 关节角度周期性变化
- 形成S形运动轨迹

**验证点**：
- ✓ 身体是否左右摆动？
- ✓ 关节角度是否周期性变化？
- ✓ 是否形成S形轨迹？

#### 场景 4: 停止 (40秒后)

**输入命令**：
- `v1 = 0.0 m/s`
- `omega1 = 0.0 rad/s`

**预期效果**：
- 机器人停止运动
- 关节角度逐渐回到0

**验证点**：
- ✓ 机器人是否停止？
- ✓ 关节角度是否回到0？

---

## 验证检查清单

### 算法正确性验证

- [ ] **圆形轨迹**：身体呈现弧形，关节角度为负值
- [ ] **直线运动**：身体变直，关节角度接近0
- [ ] **S形轨迹**：身体左右摆动，关节角度周期性变化
- [ ] **停止**：机器人停止，关节角度回到0

### 跟随效果验证

- [ ] 后续关节跟随前一个关节
- [ ] 关节角度变化平滑，无突变
- [ ] 身体形状符合预期（弧形、直线、S形）

### 实时性验证

- [ ] 响应速度：从速度命令到身体运动 < 1秒
- [ ] 更新频率：关节状态更新频率约 10 Hz
- [ ] 平滑性：运动平滑，无抖动

### 系统功能验证

- [ ] 控制器节点正常启动
- [ ] 能够接收前单元速度命令
- [ ] 能够接收关节状态
- [ ] 能够发布控制命令
- [ ] 控制命令包含正确数量的单元
- [ ] 关节角度计算合理（在合理范围内）
- [ ] 参数可以正确读取和修改
- [ ] 话题连接正常

---

## 验证结果

### 验证日期
2025-12-03

### 测试场景结果

#### 场景 1: 圆形轨迹跟踪 ✓
- **输入**: v1=0.075 m/s, omega1=0.15 rad/s
- **预期**: 第一个关节角度应该为负值（根据公式计算）
- **结果**: ✓ 通过
- **观察**: 
  - 控制命令正常发布
  - 第一个关节角度为负值（符合预期）
  - 后续关节角度跟随变化

#### 场景 2: 直线运动 ✓
- **输入**: v1=0.12 m/s, omega1=0.0 rad/s
- **预期**: 当关节角度为0时，关节角速度应该接近0
- **结果**: ✓ 通过
- **观察**:
  - 控制命令正常发布
  - 关节角度变化较小（接近0）

#### 场景 3: S形轨迹 ✓
- **输入**: v1=0.1 m/s, omega1=0.12×sin(0.3t) rad/s
- **预期**: 角速度周期性变化，关节角度响应
- **结果**: ✓ 通过
- **观察**:
  - 控制命令正常发布
  - 关节角度响应角速度变化
  - 形成S形运动轨迹

### 验证总结

#### 通过项目
1. ✓ 控制器能够接收前单元速度命令
2. ✓ 控制器能够计算并发布控制命令
3. ✓ 不同运动模式下控制器响应正确
4. ✓ 关节角度在合理范围内
5. ✓ 控制命令包含所有4个单元
6. ✓ 代码编译成功
7. ✓ 节点可以正常启动
8. ✓ 参数配置正确
9. ✓ 话题订阅和发布正常

#### 关键发现

1. **圆形轨迹跟踪**：
   - 当 omega1 > 0 时，第一个关节角度为负值
   - 这符合公式：φ̇₁ = -(2/L) v₁ sin(φ₁) - ω₁ (2 cos(φ₁) + 1)
   - 当 φ₁ = 0 时，φ̇₁ = -ω₁ * 3 = -0.45 rad/s（负值）

2. **直线运动**：
   - 当 omega1 = 0 且关节角度为0时，关节角速度接近0
   - 机器人保持直线运动

3. **S形轨迹**：
   - 角速度周期性变化时，关节角度响应平滑
   - 形成预期的S形运动轨迹

---

## 故障排除

### 问题 1: RViz 显示 "No transform from [world] to [odom]"

**原因**：Fixed Frame 设置错误

**解决方法**：
- 将 Fixed Frame 改为 `odom`
- 或发布静态 TF：
  ```bash
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world odom
  ```

### 问题 2: 模型不显示或显示不全

**原因**：robot_state_publisher 未正常运行或 URDF 加载失败

**解决方法**：
```bash
# 检查 robot_state_publisher 是否运行
ps aux | grep robot_state_publisher

# 检查 robot_description topic
ros2 topic echo /robot_description --once

# 重新启动 robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro src/asr_sdm_universe/robot/asr_sdm_description/urdf/underwater_snakerobot.urdf.xacro)"
```

### 问题 3: 关节不动或乱动

**原因**：多个 joint_states 发布源冲突

**解决方法**：
```bash
# 确保只有验证脚本在发布 joint_states
pkill -f joint_state_publisher_gui
pkill -f "ros2 topic pub /joint_states"

# 检查 joint_states 发布者
ros2 topic info /joint_states
```

### 问题 4: 控制器无响应

**原因**：控制器节点未运行或话题订阅失败

**解决方法**：
```bash
# 检查控制器节点
ros2 node list | grep asr_sdm_controller

# 检查话题连接
ros2 topic list | grep front_unit_velocity
ros2 topic list | grep joint_states

# 查看控制器日志
ros2 topic echo /asr_sdm_controller/output/controller/heartbeat
```

### 问题 5: 控制命令中的关节角度始终为0

**原因**：
- 关节状态未正确发布
- 关节名称不匹配
- 时间步长太小

**解决方法**：
```bash
# 检查关节状态话题是否正常发布
ros2 topic echo /joint_states --once

# 确认关节名称与 URDF 一致
ros2 param get /asr_sdm_controller segment_length
ros2 param get /asr_sdm_controller num_segments

# 查看控制命令
ros2 topic echo /asr_sdm_controller/output/control_cmd
```

### 问题 6: 机器人运动不连续

**原因**：
- 关节状态发布频率太低
- 控制命令更新太慢

**解决方法**：
```bash
# 检查发布频率
ros2 topic hz /joint_states
ros2 topic hz /asr_sdm_controller/output/control_cmd

# 确保 joint_states 发布频率 >= 10 Hz
# 确保控制频率设置为 10 Hz
ros2 param get /asr_sdm_controller control_frequency
```

### 问题 7: 关节角度变化太快或不稳定

**原因**：
- 速度命令过大
- 控制频率过高
- 积分时间步长不合适

**解决方法**：
- 减小速度命令（v1 < 0.2 m/s, omega1 < 0.3 rad/s）
- 调整控制频率参数（默认 10 Hz）
- 检查时间步长计算

---

## 常见问题

### Q1: 如何修改控制器参数？

**A**: 使用 ROS 2 参数设置：
```bash
# 修改段长度
ros2 param set /asr_sdm_controller segment_length 0.2

# 修改段数
ros2 param set /asr_sdm_controller num_segments 5

# 修改控制频率
ros2 param set /asr_sdm_controller control_frequency 20.0
```

### Q2: 如何查看所有可用参数？

**A**: 
```bash
ros2 param list /asr_sdm_controller
ros2 param get /asr_sdm_controller segment_length
ros2 param get /asr_sdm_controller num_segments
ros2 param get /asr_sdm_controller control_frequency
```

### Q3: 如何停止所有进程？

**A**: 
```bash
pkill -f robot_state_publisher
pkill -f asr_sdm_controller
pkill -f verify_head_tracking_rviz.py
pkill -f rviz2
pkill -f joint_state_publisher
```

### Q4: 验证脚本运行多长时间？

**A**: 验证脚本自动运行 45 秒，包含 4 个测试场景：
- 0-15秒：圆形轨迹
- 15-25秒：直线运动
- 25-40秒：S形轨迹
- 40秒后：停止

### Q5: 如何重新运行验证？

**A**: 直接重新运行验证脚本：
```bash
cd /home/wwlwwl/asr_sdm_ws
source install/setup.bash
python3 src/asr_sdm_universe/control/asr_sdm_controller/verify_head_tracking_rviz.py
```

### Q6: 如何验证公式正确性？

**A**: 根据论文公式验证：
```
φ̇₁ = -(2/L) v₁ sin(φ₁) - ω₁ (2 cos(φ₁) + 1)
```

当 `φ₁ = 0`, `v₁ = 0.075`, `omega1 = 0.15`, `L = 0.18` 时：
```
φ̇₁ = -(2/0.18) * 0.075 * sin(0) - 0.15 * (2*cos(0) + 1)
   = 0 - 0.15 * 3
   = -0.45 rad/s
```

在 RViz 中观察第一个关节角度是否按此速度变化。

---

## 性能指标

### 响应时间
- 控制命令发布频率：应该接近设定的 control_frequency（默认10 Hz）
- 从速度命令到控制命令输出的延迟：< 100ms

### 稳定性
- 关节角度应该平滑变化，无突变
- 长时间运行应该稳定
- CPU 使用率应该较低（< 5%）

### 准确性
- 关节角度应该在合理范围内（-π 到 π）
- 响应应该符合预期（根据公式计算）
- 控制命令包含正确数量的单元（4个）

### 实时性
- 响应速度：从速度命令到身体运动 < 1秒
- 更新频率：关节状态更新频率约 10 Hz
- 平滑性：运动平滑，无抖动

---

## 下一步

验证通过后，可以：

1. **集成到完整的机器人系统中**
2. **在实际硬件上测试**
3. **调整控制参数以优化性能**
4. **添加轨迹规划功能**
5. **实现更复杂的跟踪算法**
6. **添加速度限制和加速度限制**
7. **实现反馈控制以提高跟踪精度**

---

## 参考

- 控制器节点：`src/asr_sdm_universe/control/asr_sdm_controller/src/asr_sdm_controller_node.cpp`
- 控制器算法：`src/asr_sdm_universe/control/asr_sdm_controller/src/front_unit_following_controller.cpp`
- 验证脚本：`src/asr_sdm_universe/control/asr_sdm_controller/verify_head_tracking_rviz.py`
- URDF 模型：`src/asr_sdm_universe/robot/asr_sdm_description/urdf/underwater_snakerobot.urdf.xacro`

---

**文档版本**: 1.0  
**最后更新**: 2025-12-08

