#!/usr/bin/env python3
"""
测试脚本：验证 front-unit-following controller 节点功能

使用方法：
1. 在一个终端启动控制器节点：
   ros2 run asr_sdm_controller asr_sdm_controller

2. 在另一个终端运行此脚本：
   python3 test_controller_node.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from asr_sdm_control_msgs.msg import ControlCmd
import time


class ControllerTester(Node):
    def __init__(self):
        super().__init__('controller_tester')
        
        # 发布前单元速度命令
        self.vel_pub = self.create_publisher(
            Twist,
            '/asr_sdm_controller/input/front_unit_velocity',
            10
        )
        
        # 发布关节状态
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # 订阅控制命令输出
        self.control_cmd_sub = self.create_subscription(
            ControlCmd,
            '/asr_sdm_controller/output/control_cmd',
            self.control_cmd_callback,
            10
        )
        
        self.control_cmd_received = False
        self.last_control_cmd = None
        
        self.get_logger().info('Controller tester initialized')
    
    def control_cmd_callback(self, msg):
        self.control_cmd_received = True
        self.last_control_cmd = msg
        self.get_logger().info(
            f'Received control command with {len(msg.units_cmd)} units'
        )

    def wait_for_control_cmd(self, joint_angles, timeout=1.0):
        """
        在给定超时时间内等待控制命令，同时重复发布关节状态，确保控制器有输入。
        """
        start = time.time()
        while time.time() - start < timeout:
            # 持续发布关节状态（10 Hz 左右）
            self.publish_joint_state(joint_angles)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
            if self.control_cmd_received:
                return True
        return False
    
    def publish_joint_state(self, joint_angles):
        """发布关节状态"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'
        
        # 根据 URDF 中的关节名称
        joint_names = [
            'base_link_to_link_2',
            'link_2_to_link_3',
            'link_3_to_link_4',
            'link_4_to_link_5'
        ]
        
        for i, name in enumerate(joint_names):
            joint_state.name.append(name)
            if i < len(joint_angles):
                joint_state.position.append(joint_angles[i])
            else:
                joint_state.position.append(0.0)
            joint_state.velocity.append(0.0)
            joint_state.effort.append(0.0)
        
        self.joint_state_pub.publish(joint_state)
    
    def publish_velocity_command(self, v1, omega1):
        """发布前单元速度命令"""
        twist = Twist()
        twist.linear.x = v1
        twist.angular.z = omega1
        self.vel_pub.publish(twist)
        self.get_logger().info(f'Published velocity: v1={v1}, omega1={omega1}')
    
    def test_scenario(self, name, v1, omega1, joint_angles):
        """测试场景"""
        self.get_logger().info(f'\n=== Testing: {name} ===')
        
        # 重置标志
        self.control_cmd_received = False
        self.last_control_cmd = None
        
        # 发布关节状态
        self.publish_joint_state(joint_angles)
        time.sleep(0.1)
        
        # 发布速度命令
        self.publish_velocity_command(v1, omega1)
        received = self.wait_for_control_cmd(joint_angles, timeout=1.5)

        # 检查是否收到控制命令
        if received and self.last_control_cmd:
            self.get_logger().info(f'✓ Test passed: {name}')
            self.get_logger().info(
                f'  Units in command: {len(self.last_control_cmd.units_cmd)}'
            )
            for i, unit_cmd in enumerate(self.last_control_cmd.units_cmd):
                self.get_logger().info(
                    f'  Unit {unit_cmd.unit_id}: '
                    f'joint1_angle={unit_cmd.joint1_angle}, '
                    f'joint2_angle={unit_cmd.joint2_angle}'
                )
            return True
        else:
            self.get_logger().warn(f'✗ Test failed: {name} - No control command received')
            return False


def main():
    rclpy.init()
    
    tester = ControllerTester()
    
    # 等待节点初始化
    time.sleep(1.0)
    
    # 测试场景
    test_results = []
    
    # 测试1: 静止状态
    test_results.append(tester.test_scenario(
        'Zero velocities, straight joints',
        v1=0.0,
        omega1=0.0,
        joint_angles=[0.0, 0.0, 0.0, 0.0]
    ))
    
    # 测试2: 前进运动
    test_results.append(tester.test_scenario(
        'Forward motion',
        v1=0.1,
        omega1=0.0,
        joint_angles=[0.0, 0.0, 0.0, 0.0]
    ))
    
    # 测试3: 旋转运动
    test_results.append(tester.test_scenario(
        'Rotation only',
        v1=0.0,
        omega1=0.1,
        joint_angles=[0.0, 0.0, 0.0, 0.0]
    ))
    
    # 测试4: 弯曲配置
    test_results.append(tester.test_scenario(
        'Curved configuration',
        v1=0.1,
        omega1=0.05,
        joint_angles=[0.3, 0.2, 0.1, 0.0]
    ))
    
    # 测试5: 大角度弯曲
    test_results.append(tester.test_scenario(
        'Large angle curve',
        v1=0.15,
        omega1=0.1,
        joint_angles=[0.5, 0.4, 0.3, 0.2]
    ))
    
    # 总结
    passed = sum(test_results)
    total = len(test_results)
    tester.get_logger().info(
        f'\n=== Test Summary ===\n'
        f'Passed: {passed}/{total}\n'
        f'Success rate: {100*passed/total:.1f}%'
    )
    
    rclpy.shutdown()
    return 0 if passed == total else 1


if __name__ == '__main__':
    exit(main())

