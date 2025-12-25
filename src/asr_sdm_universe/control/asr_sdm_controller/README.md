# ASR SDM Controller

蛇形机器人前端单元跟随控制器，支持 Pinocchio 精确运动学计算。

## 概述

本控制器实现了蛇形机器人的前端单元跟随控制算法，支持两种计算模式：
- **简化模型**：快速计算，适用于实时控制
- **Pinocchio 精确计算**：基于 Pinocchio 库的精确运动学计算，适用于高精度场景

## 快速开始

### 安装

```bash
cd /home/wwlwwl/asr_sdm_ws

# 编译两个包
colcon build --packages-select asr_sdm_description asr_sdm_controller

# Source 环境
source install/setup.bash
```

### 基本使用

```bash
# 使用简化模型（默认）
ros2 run asr_sdm_controller asr_sdm_controller

# 使用 Pinocchio 精确计算
ros2 run asr_sdm_controller asr_sdm_controller \
  --ros-args -p use_pinocchio:=true
```

### 参数配置

控制器支持以下 ROS 2 参数：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `use_pinocchio` | bool | `false` | 是否使用 Pinocchio 进行精确计算 |
| `robot_description` | string | - | URDF 机器人描述（通常通过 launch 文件设置） |
| `control_frequency` | double | `100.0` | 控制频率 (Hz) |

## ROS 2 接口

### 订阅话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/asr_sdm_controller/input/front_unit_velocity` | `geometry_msgs/Twist` | 前端单元速度命令（线速度和角速度） |
| `/joint_states` | `sensor_msgs/JointState` | 当前关节状态反馈 |

### 发布话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/asr_sdm_controller/output/control_cmd` | `asr_sdm_control_msgs/ControlCmd` | 输出控制命令到硬件层 |
| `/asr_sdm_controller/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 控制器诊断信息 |
| `/asr_sdm_controller/computation_method` | `std_msgs/String` | 当前使用的计算方法（"simplified" 或 "pinocchio"） |
| `/asr_sdm_controller/computation_time` | `std_msgs/Float64` | 单次计算耗时（秒） |

## 可视化与测试

### 完整启动流程（多终端方式）

推荐使用多个终端分别启动各个组件，便于观察日志和调试：

```bash
cd /home/wwlwwl/asr_sdm_ws

# 编译两个包
colcon build --packages-select asr_sdm_description asr_sdm_controller

# Source 环境
source install/setup.bash

# 终端1：启动 RViz + robot_state_publisher
ros2 launch asr_sdm_description horizontal_snake_visualization.launch.py

# 终端2：启动 C++ 控制器节点
ros2 run asr_sdm_controller asr_sdm_controller

# 终端3：发布速度命令（或运行验证脚本）
python3 src/asr_sdm_universe/control/asr_sdm_controller/verify_head_tracking_rviz.py
```

### 一键运行

使用提供的脚本快速启动完整环境：

```bash
cd /home/wwlwwl/asr_sdm_ws
source install/setup.bash
./src/asr_sdm_universe/control/asr_sdm_controller/start_horizontal_visualization.sh
```

该脚本会：
1. 启动 RViz2 可视化界面
2. 启动 robot_state_publisher
3. 运行验证脚本发布测试命令

### 手动测试

```bash
# 终端 1: 启动控制器
ros2 run asr_sdm_controller asr_sdm_controller

# 终端 2: 发布测试速度命令
ros2 topic pub /asr_sdm_controller/input/front_unit_velocity \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}" \
  -r 10

# 终端 3: 查看控制命令输出
ros2 topic echo /asr_sdm_controller/output/control_cmd

# 终端 4: 监控计算时间
ros2 topic echo /asr_sdm_controller/computation_time
```

### 测试覆盖

控制器已通过以下测试验证：
- ✅ URDF 加载 (100 次迭代)
- ✅ 前向运动学精度 (100 次迭代)
- ✅ 雅可比计算 (50 次迭代)
- ✅ 控制律验证 (100 次迭代)
- ✅ Pinocchio 集成 (50 次迭代)

## 故障排除

### Pinocchio 初始化失败

如果遇到 Pinocchio 初始化错误：

```bash
# 检查 URDF 参数是否设置
ros2 param list | grep robot_description

# 验证 Pinocchio 安装
python3 -c "import pinocchio; print(pinocchio.__version__)"

# 检查 URDF 文件是否存在
ros2 param get /asr_sdm_controller robot_description
```

### 控制命令无输出

1. 检查是否订阅到关节状态：
   ```bash
   ros2 topic echo /joint_states
   ```

2. 检查是否接收到速度命令：
   ```bash
   ros2 topic echo /asr_sdm_controller/input/front_unit_velocity
   ```

3. 查看诊断信息：
   ```bash
   ros2 topic echo /asr_sdm_controller/diagnostics
   ```

### 计算时间过长

- 如果使用 Pinocchio 模式计算时间过长，考虑切换到简化模型
- 检查系统负载和 CPU 使用率
- 查看 `/asr_sdm_controller/computation_time` 话题监控实际计算时间

### 清理节点

当需要停止所有相关节点时，可以使用以下命令：

```bash
pkill -f asr_sdm_controller
pkill -f static_transform_publisher
pkill -f verify_head_tracking_rviz.py
pkill -f rviz2
```

或者使用 `Ctrl+C` 逐个停止各个终端中的进程。

## 依赖

### 系统依赖

- ROS 2 (Jazzy/Jazzy Jalisco)
- Pinocchio >= 2.6.0
- Eigen3

### ROS 2 依赖

- `rclcpp`
- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `diagnostic_msgs`
- `asr_sdm_control_msgs`
- `asr_sdm_hardware_msgs`

## 参考文献

Fukushima, H., et al. (2012). "Modeling and Control of a Snake-Like Robot Using the Screw-Drive Mechanism." *IEEE Transactions on Robotics*, 28(3), 541-554.
