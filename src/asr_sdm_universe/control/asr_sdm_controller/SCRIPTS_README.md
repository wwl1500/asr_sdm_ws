# 脚本使用说明

本文档说明控制器包中保留的脚本文件及其用途。

## 保留的脚本

### 1. `verify_head_tracking_rviz.py` ⭐ **主要验证脚本**

**用途**：完整的头部跟踪控制算法验证脚本，包含RViz可视化、TF发布和轨迹绘制。

**功能**：
- 自动运行4个测试场景（圆形轨迹、直线运动、S形轨迹、停止）
- 发布前单元速度命令到控制器
- 发布关节状态（包含主体关节和螺旋桨关节）
- 发布 `odom->base_link` TF，实现机器人平移可视化
- 发布 `/head_path` Marker，显示头部轨迹线
- 自动运行45秒后完成

**使用方法**：
```bash
cd /home/wwlwwl/asr_sdm_ws
source install/setup.bash

# 终端1: 启动控制器
ros2 run asr_sdm_controller asr_sdm_controller

# 终端2: 启动 robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro src/asr_sdm_universe/robot/asr_sdm_description/urdf/underwater_snakerobot.urdf.xacro)"

# 终端3: 启动 RViz
rviz2
# 在 RViz 中设置 Fixed Frame=odom，添加 RobotModel 和 Marker(/head_path)

# 终端4: 运行验证脚本
python3 src/asr_sdm_universe/control/asr_sdm_controller/verify_head_tracking_rviz.py
```

**输出**：
- 控制台输出：各阶段的关节角度和速度命令
- RViz可视化：机器人模型在水平面内弯曲，显示轨迹路径

---

### 2. `test_controller_node.py`

**用途**：测试控制器节点的基本功能，验证控制器能否正确响应不同的输入场景。

**功能**：
- 测试5个场景：静止、前进、旋转、弯曲配置、大角度弯曲
- 验证控制器能否接收速度命令和关节状态
- 验证控制器能否发布控制命令
- 显示测试结果统计

**使用方法**：
```bash
cd /home/wwlwwl/asr_sdm_ws
source install/setup.bash

# 终端1: 启动控制器
ros2 run asr_sdm_controller asr_sdm_controller

# 终端2: 运行测试脚本
python3 src/asr_sdm_universe/control/asr_sdm_controller/test_controller_node.py
```

**输出**：
- 每个测试场景的通过/失败状态
- 控制命令的详细信息
- 最终测试总结（通过率）

---

### 3. `test_front_unit_following_controller.cpp`

**用途**：C++单元测试，测试控制器算法的正确性（不依赖ROS 2节点）。

**使用方法**：
```bash
cd /home/wwlwwl/asr_sdm_ws
colcon build --packages-select asr_sdm_controller
source install/setup.bash

# 运行单元测试
ros2 run asr_sdm_controller test_front_unit_following_controller

# 或使用 colcon test
colcon test --packages-select asr_sdm_controller
colcon test-result --verbose
```

---

## 已删除的脚本（功能已整合）

以下脚本已被删除，其功能已整合到 `verify_head_tracking_rviz.py` 或文档中：

- ❌ `launch_rviz_visualization.sh` - 功能已被 `verify_head_tracking_rviz.py` 替代
- ❌ `quick_test.sh` - 功能重复，使用说明已整合到文档
- ❌ `quick_verify_head_tracking.sh` - 功能简单，直接使用 `verify_head_tracking_rviz.py` 即可
- ❌ `restart_rviz_verification.sh` - 功能重复，使用说明已整合到文档
- ❌ `test_head_tracking_simple.sh` - 功能已被 `verify_head_tracking_rviz.py` 替代
- ❌ `test_head_tracking.py` - 功能已被 `verify_head_tracking_rviz.py` 替代

---

## 推荐使用流程

### 快速验证（推荐）

使用 `verify_head_tracking_rviz.py` 进行完整的可视化验证：

1. 按照 `HEAD_TRACKING_CONTROL_GUIDE.md` 中的详细步骤操作
2. 在RViz中观察机器人运动
3. 查看轨迹路径和关节角度变化

### 单元测试

使用 `test_controller_node.py` 进行功能测试：

1. 启动控制器节点
2. 运行测试脚本
3. 查看测试结果

### 算法测试

使用 `test_front_unit_following_controller.cpp` 进行算法正确性测试：

1. 编译包
2. 运行单元测试
3. 查看测试结果

---

## 相关文档

- **完整指南**：`HEAD_TRACKING_CONTROL_GUIDE.md` - 包含详细的使用说明、验证步骤和故障排除
- **代码实现**：`src/front_unit_following_controller.cpp` - 控制器算法实现
- **节点实现**：`src/asr_sdm_controller_node.cpp` - ROS 2节点实现

---

**最后更新**：2025-12-08

