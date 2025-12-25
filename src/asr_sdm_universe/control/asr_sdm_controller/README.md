# ASR SDM Controller

蛇形机器人前端单元跟随控制器，支持 Pinocchio 精确运动学计算。

## 功能特性

- ✅ **前端单元跟随控制**：实现论文中的控制律
- ✅ **Pinocchio 集成**：精确的运动学和雅可比计算
- ✅ **自动回退**：Pinocchio 失败时自动回退到简化模型
- ✅ **实时诊断**：性能监控和状态报告
- ✅ **错误处理**：完整的输入验证和超时检查
- ✅ **向后兼容**：与现有系统完全兼容

## 快速开始

### 安装

```bash
cd ~/asr_sdm_ws
colcon build --packages-select asr_sdm_controller
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

## 文档

- [Pinocchio 完整指南](docs/PINOCCHIO_COMPLETE_GUIDE.md) - 详细的使用说明、接口文档、故障排除

## 主要参数

| 参数 | 默认值 | 描述 |
|-----|--------|------|
| `use_pinocchio` | false | 启用 Pinocchio |
| `segment_length` | 0.18 | 段长度 (m) |
| `num_segments` | 4 | 段数量 |
| `control_frequency` | 10.0 | 控制频率 (Hz) |
| `enable_diagnostics` | true | 启用诊断 |

完整参数列表请参见 [完整指南](docs/PINOCCHIO_COMPLETE_GUIDE.md#配置参数)。

## ROS 2 接口

### 订阅

- `/asr_sdm_controller/input/front_unit_velocity` (geometry_msgs/Twist) - 前端速度命令
- `/joint_states` (sensor_msgs/JointState) - 关节状态

### 发布

- `/asr_sdm_controller/output/control_cmd` (asr_sdm_control_msgs/ControlCmd) - 控制命令
- `/asr_sdm_controller/diagnostics` (diagnostic_msgs/DiagnosticArray) - 诊断信息
- `/asr_sdm_controller/computation_method` (std_msgs/String) - 计算方法
- `/asr_sdm_controller/computation_time` (std_msgs/Float64) - 计算时间

## 测试

```bash
# 运行所有测试
colcon test --packages-select asr_sdm_controller

# 查看测试结果
colcon test-result --all --verbose
```

测试覆盖：
- ✅ URDF 加载 (100 次迭代)
- ✅ 前向运动学精度 (100 次迭代)
- ✅ 雅可比计算 (50 次迭代)
- ✅ 控制律验证 (100 次迭代)
- ✅ Pinocchio 集成 (50 次迭代)

## 性能

| 指标 | 简化模型 | Pinocchio |
|-----|---------|-----------|
| 计算时间 | < 0.1 ms | < 1.0 ms |
| CPU 使用 | ~5% | ~8% |
| 精度 | 近似 | 精确 |

## 示例

### Launch 文件示例

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='asr_sdm_controller',
            executable='asr_sdm_controller',
            parameters=[{
                'use_pinocchio': True,
                'enable_diagnostics': True,
                'segment_length': 0.18,
                'num_segments': 4,
            }]
        ),
    ])
```

### Python 示例

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher = self.create_publisher(
            Twist,
            '/asr_sdm_controller/input/front_unit_velocity',
            10
        )
        self.timer = self.create_timer(0.1, self.publish_velocity)
    
    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.1  # 0.1 m/s
        msg.angular.z = 0.05  # 0.05 rad/s
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = VelocityPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
```

## 故障排除

### Pinocchio 初始化失败

```bash
# 检查 URDF 参数
ros2 param list | grep robot_description

# 验证 Pinocchio 安装
python3 -c "import pinocchio; print(pinocchio.__version__)"
```

### 速度警告

```bash
# 调整警告阈值
ros2 param set /asr_sdm_controller velocity_warning_threshold 3.0
```

更多故障排除信息请参见 [完整指南](docs/PINOCCHIO_COMPLETE_GUIDE.md#故障排除)。

## 依赖

- ROS 2 Jazzy
- Pinocchio >= 2.6.0
- Eigen3
- rclcpp
- geometry_msgs
- sensor_msgs
- diagnostic_msgs
- asr_sdm_control_msgs
- asr_sdm_hardware_msgs

## 贡献

欢迎贡献！请确保：
1. 所有测试通过
2. 代码符合 ROS 2 风格指南
3. 添加适当的文档

## 许可证

TODO: 添加许可证信息

## 参考文献

Fukushima, H., et al. (2012). "Modeling and Control of a Snake-Like Robot Using the Screw-Drive Mechanism." IEEE Transactions on Robotics, 28(3), 541-554.

## 联系方式

- 维护者：[您的名字]
- 邮箱：[您的邮箱]
- 项目主页：[项目链接]
