#!/usr/bin/env python3
"""
RViz 头部跟踪算法验证脚本

这个脚本会发布不同的头部运动命令，让你在 RViz 中观察机器人身体的跟随效果。

使用方法：
1. 确保 RViz 和控制器都在运行
2. 运行此脚本：python3 verify_head_tracking_rviz.py
3. 在 RViz 中观察机器人身体的运动
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time
import math
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster


class HeadTrackingVerifier(Node):
    def __init__(self):
        super().__init__('head_tracking_verifier')
        
        # 发布前单元速度命令
        self.vel_pub = self.create_publisher(
            Twist,
            '/asr_sdm_controller/input/front_unit_velocity',
            10
        )
        
        # 发布关节状态（初始状态）
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        # 发布 odom->base_link TF 及轨迹
        self.tf_broadcaster = TransformBroadcaster(self)
        self.path_pub = self.create_publisher(Marker, '/head_path', 10)
        
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]
        self.time_elapsed = 0.0
        self.dt = 0.1

        # 平面位姿（odom 坐标系）
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.path_points = []
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('头部跟踪算法验证脚本已启动')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        self.get_logger().info('验证场景：')
        self.get_logger().info('  1. 圆形轨迹跟踪 (0-15秒)')
        self.get_logger().info('  2. 直线运动 (15-25秒)')
        self.get_logger().info('  3. S形轨迹 (25-40秒)')
        self.get_logger().info('  4. 停止 (40秒后)')
        self.get_logger().info('')
        self.get_logger().info('请在 RViz 中观察机器人身体的运动！')
        self.get_logger().info('=' * 60)
    
    def publish_joint_state(self):
        """发布当前关节状态"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'
        
        # 主体关节（会随控制算法更新）
        main_joint_names = [
            'base_link_to_link_2',
            'link_2_to_link_3',
            'link_3_to_link_4',
            'link_4_to_link_5'
        ]

        # 螺旋桨关节（保持为 0，用于补全 TF）
        screw_joint_names = [
            'base_link_to_screw_1_left_joint',
            'base_link_to_screw_1_right_joint',
            'link_2_to_screw_2_left_joint',
            'link_2_to_screw_2_right_joint',
            'link_3_to_screw_3_left_joint',
            'link_3_to_screw_3_right_joint',
            'link_4_to_screw_4_left_joint',
            'link_4_to_screw_4_right_joint',
            'link_5_to_screw_5_left_joint',
            'link_5_to_screw_5_right_joint',
        ]

        joint_names = main_joint_names + screw_joint_names

        # 主体关节使用当前角度，螺旋桨关节填 0
        for i, name in enumerate(joint_names):
            joint_state.name.append(name)
            if i < len(self.joint_angles):
                joint_state.position.append(self.joint_angles[i])
            else:
                joint_state.position.append(0.0)
            joint_state.velocity.append(0.0)
            joint_state.effort.append(0.0)
        
        self.joint_state_pub.publish(joint_state)
    
    def publish_velocity_command(self, v1, omega1, description=""):
        """发布前单元速度命令"""
        twist = Twist()
        twist.linear.x = v1
        twist.angular.z = omega1
        self.vel_pub.publish(twist)
        if description:
            self.get_logger().info(f'[{self.time_elapsed:.1f}s] {description}: v1={v1:.3f} m/s, omega1={omega1:.3f} rad/s')
    
    def update_joint_angles(self, v1, omega1):
        """
        根据控制算法更新关节角度（简化版本）
        实际应该从控制命令中获取，这里用公式近似
        """
        L = 0.18  # 段长度
        dt = 0.1  # 时间步长
        
        # 第一个关节的角速度（根据论文公式）
        if len(self.joint_angles) > 0:
            phi1 = self.joint_angles[0]
            phi_dot1 = -(2.0 / L) * v1 * math.sin(phi1) - omega1 * (2.0 * math.cos(phi1) + 1.0)
            self.joint_angles[0] += phi_dot1 * dt
        
        # 后续关节（简化处理）
        for i in range(1, len(self.joint_angles)):
            # 简化的跟随模型
            if i > 0:
                prev_phi = self.joint_angles[i-1]
                # 后续关节跟随前一个关节
                self.joint_angles[i] = 0.9 * self.joint_angles[i] + 0.1 * prev_phi * 0.5

    def publish_tf_and_path(self, v1, omega1):
        """积分前段位姿，发布 odom->base_link TF 与头部轨迹线"""
        dt = self.dt
        # 2D 位姿积分
        self.x += v1 * math.cos(self.theta) * dt
        self.y += v1 * math.sin(self.theta) * dt
        self.theta += omega1 * dt

        # TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        qz = math.sin(self.theta * 0.5)
        qw = math.cos(self.theta * 0.5)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # 路径 Marker（LINE_STRIP）
        p = Point()
        p.x = self.x
        p.y = self.y
        p.z = 0.0
        self.path_points.append(p)

        marker = Marker()
        marker.header.stamp = t.header.stamp
        marker.header.frame_id = 'odom'
        marker.ns = 'head_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # 线宽
        marker.color = ColorRGBA(r=0.1, g=0.8, b=0.1, a=0.8)
        marker.points = self.path_points
        self.path_pub.publish(marker)
    
    def timer_callback(self):
        self.time_elapsed += 0.1
        t = self.time_elapsed
        
        # 发布关节状态
        self.publish_joint_state()
        
        # 场景 1: 圆形轨迹跟踪 (0-15秒)
        if t < 15.0:
            radius = 0.5  # 半径 0.5m
            angular_vel = 0.15  # 角速度 0.15 rad/s
            v1 = radius * angular_vel  # 线速度
            omega1 = angular_vel
            if int(t * 10) % 50 == 0:  # 每5秒打印一次
                self.publish_velocity_command(v1, omega1, "圆形轨迹跟踪")
            else:
                self.publish_velocity_command(v1, omega1)
            self.update_joint_angles(v1, omega1)
        
        # 场景 2: 直线运动 (15-25秒)
        elif t < 25.0:
            v1 = 0.12  # 0.12 m/s
            omega1 = 0.0
            if int(t * 10) % 50 == 0:
                self.publish_velocity_command(v1, omega1, "直线运动")
            else:
                self.publish_velocity_command(v1, omega1)
            self.update_joint_angles(v1, omega1)
        
        # 场景 3: S形轨迹 (25-40秒)
        elif t < 40.0:
            v1 = 0.1
            omega1 = 0.12 * math.sin(0.3 * (t - 25.0))  # 正弦变化的角速度
            if int(t * 10) % 50 == 0:
                self.publish_velocity_command(v1, omega1, "S形轨迹")
            else:
                self.publish_velocity_command(v1, omega1)
            self.update_joint_angles(v1, omega1)
        
        # 场景 4: 停止 (40秒后)
        else:
            v1 = 0.0
            omega1 = 0.0
            if int(t * 10) % 50 == 0:
                self.publish_velocity_command(v1, omega1, "停止")
            else:
                self.publish_velocity_command(v1, omega1)
            # 关节角度逐渐回到0
            for i in range(len(self.joint_angles)):
                self.joint_angles[i] *= 0.95

        # 发布 TF & 轨迹
        self.publish_tf_and_path(v1, omega1)

        # 每10秒输出一次状态
        if int(t * 10) % 100 == 0:
            self.get_logger().info(
                f'[{t:.1f}s] 关节角度: '
                f'[{", ".join(f"{a:.3f}" for a in self.joint_angles)}]'
            )


def main():
    rclpy.init()
    
    verifier = HeadTrackingVerifier()
    
    try:
        # 运行45秒
        import threading
        stop_event = threading.Event()
        
        def stop_after_timeout():
            time.sleep(45.0)
            stop_event.set()
        
        timeout_thread = threading.Thread(target=stop_after_timeout)
        timeout_thread.start()
        
        while not stop_event.is_set():
            rclpy.spin_once(verifier, timeout_sec=0.1)
        
        verifier.get_logger().info('')
        verifier.get_logger().info('=' * 60)
        verifier.get_logger().info('验证完成！')
        verifier.get_logger().info('=' * 60)
        
    except KeyboardInterrupt:
        verifier.get_logger().info('用户中断')
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    exit(main())

