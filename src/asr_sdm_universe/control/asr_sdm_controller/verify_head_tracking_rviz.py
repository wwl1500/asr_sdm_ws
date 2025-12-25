#!/usr/bin/env python3
"""
RViz 水平面头部跟踪算法验证脚本

这个脚本会发布不同的头部运动命令，让你在 RViz 中观察机器人在水平面上的跟随效果。
link_5 作为头部单元，在水平面（XY平面）上运动。

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
        self.get_logger().info('水平面头部跟踪算法验证脚本已启动')
        self.get_logger().info('link_5 作为头部单元在水平面（XY平面）运动')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        self.get_logger().info('验证场景：')
        self.get_logger().info('  1. 圆形轨迹跟踪 (0-15秒)')
        self.get_logger().info('  2. 直线运动 (15-25秒)')
        self.get_logger().info('  3. S形轨迹 (25-55秒)')
        self.get_logger().info('  4. 停止 (55秒后)')
        self.get_logger().info('')
        self.get_logger().info('请在 RViz 中观察机器人在水平面上的运动！')
        self.get_logger().info('=' * 60)
    
    def publish_joint_state(self):
        """发布当前关节状态"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'
        
        # 主体关节（会随控制算法更新）
        # 注意：螺旋桨关节是 fixed 类型，由 robot_state_publisher 自动处理
        joint_names = [
            'base_link_to_link_2',
            'link_2_to_link_3',
            'link_3_to_link_4',
            'link_4_to_link_5'
        ]

        # 发布主体关节状态
        for i, name in enumerate(joint_names):
            joint_state.name.append(name)
            joint_state.position.append(self.joint_angles[i])
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
        根据头部角速度更新关节角度
        
        蛇形机器人的身体跟随原理：
        - 当头部转弯时，身体各段应该跟随头部的轨迹
        - 关节角度应该使身体弯向转弯方向
        - 使用简化的跟随模型：关节角度与头部角速度成正比
        """
        dt = 0.1  # 时间步长
        
        # 关节角度跟随头部角速度
        # 当 omega1 > 0（左转）时，关节角度应该为正（身体向左弯）
        # joint_angles[3] 是最靠近头部的关节（link_4->link_5）
        # joint_angles[0] 是最靠近尾部的关节（base_link->link_2）
        
        # 头部附近的关节响应更快，尾部关节响应更慢
        gain = [0.3, 0.4, 0.5, 0.6]  # 从尾部到头部，增益递增
        
        for i in range(len(self.joint_angles)):
            # 目标角度与角速度成正比
            target_angle = omega1 * gain[i] * 2.0
            # 平滑过渡到目标角度
            self.joint_angles[i] = 0.9 * self.joint_angles[i] + 0.1 * target_angle
            # 限制关节角度范围
            self.joint_angles[i] = max(-0.5, min(0.5, self.joint_angles[i]))

    def compute_base_from_head(self, head_x, head_y, head_theta):
        """
        从头部（link_5）位置反推 base_link 的位置和朝向
        使用逆向运动学：从 link_5 沿着关节链反向计算到 base_link
        """
        L = 0.18  # 段长度
        
        # 从 link_5 开始，反向遍历
        x = head_x
        y = head_y
        theta = head_theta
        
        # 反向遍历关节（从 link_5 到 base_link）
        # joint_angles[3] 是 link_4->link_5 的关节
        # joint_angles[0] 是 base_link->link_2 的关节
        for i in range(len(self.joint_angles) - 1, -1, -1):
            phi = self.joint_angles[i]
            # 先反向旋转关节角度
            theta -= phi
            # 然后反向移动一个段长度
            x -= L * math.cos(theta)
            y -= L * math.sin(theta)
        
        return x, y, theta
    
    def publish_tf_and_path(self, v1, omega1):
        """
        积分头部位姿，发布 odom->base_link TF 与头部轨迹线
        
        策略：
        1. 直接积分头部（link_5）的位置和朝向
        2. 从头部位置反推 base_link 的位置
        3. 绿色轨迹直接使用头部位置
        """
        dt = self.dt
        
        # 2D 位姿积分（头部 link_5 的位姿）
        self.x += v1 * math.cos(self.theta) * dt
        self.y += v1 * math.sin(self.theta) * dt
        self.theta += omega1 * dt

        # 从头部位置反推 base_link 的位置和朝向
        base_x, base_y, base_theta = self.compute_base_from_head(
            self.x, self.y, self.theta
        )

        now = self.get_clock().now().to_msg()
        
        # TF: world -> odom（静态变换，world 和 odom 重合）
        t_world = TransformStamped()
        t_world.header.stamp = now
        t_world.header.frame_id = 'world'
        t_world.child_frame_id = 'odom'
        t_world.transform.translation.x = 0.0
        t_world.transform.translation.y = 0.0
        t_world.transform.translation.z = 0.0
        t_world.transform.rotation.x = 0.0
        t_world.transform.rotation.y = 0.0
        t_world.transform.rotation.z = 0.0
        t_world.transform.rotation.w = 1.0
        
        # TF: odom -> base_link（使用反推的位置和朝向）
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = base_x
        t.transform.translation.y = base_y
        t.transform.translation.z = 0.0
        
        # 绕 Z 轴旋转（base_link 的朝向）
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(base_theta / 2)
        t.transform.rotation.w = math.cos(base_theta / 2)
        
        # 发布两个 TF
        self.tf_broadcaster.sendTransform([t_world, t])

        # 路径 Marker（LINE_STRIP）- 直接使用头部位置
        p = Point()
        p.x = self.x
        p.y = self.y
        p.z = 0.0
        self.path_points.append(p)

        marker = Marker()
        marker.header.stamp = now
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
        
        # 场景 3: S形轨迹 (25-55秒) - 延长到30秒
        elif t < 55.0:
            v1 = 0.1
            omega1 = 0.12 * math.sin(0.3 * (t - 25.0))  # 正弦变化的角速度
            if int(t * 10) % 50 == 0:
                self.publish_velocity_command(v1, omega1, "S形轨迹")
            else:
                self.publish_velocity_command(v1, omega1)
            self.update_joint_angles(v1, omega1)
        
        # 场景 4: 停止 (55秒后)
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
        # 运行60秒
        import threading
        stop_event = threading.Event()
        
        def stop_after_timeout():
            time.sleep(60.0)
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

