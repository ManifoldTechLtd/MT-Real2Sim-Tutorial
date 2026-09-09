#!/usr/bin/env python3
"""
虚拟轴距标定工具
用于标定 AL-iLQR Go2 的 virtual_wheelbase 参数。

使用方法：
  python3 calibrate_wheelbase.py

原理：
  1. 发送已知的 cmd_vel（固定线速度 + 角速度）
  2. 记录里程计反馈的实际角速度
  3. 计算期望 vs 实际的比值
  4. 推算正确的 virtual_wheelbase
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import sys


class WheelbaseCalibrator:
    def __init__(self):
        rospy.init_node('wheelbase_calibrator', anonymous=True)
        
        # 参数
        self.test_linear_vel = rospy.get_param('~test_linear_vel', 0.0)  # 原地转
        self.test_angular_vel = rospy.get_param('~test_angular_vel', 0.5)  # 期望角速度
        self.test_duration = rospy.get_param('~test_duration', 10.0)  # 测试时长 s
        self.odom_topic = rospy.get_param('~odom_topic', '/odin1/odometry')
        
        # 数据记录
        self.angular_velocities = []
        self.recording = False
        
        # ROS 接口
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        
    def odom_callback(self, msg):
        if self.recording:
            omega = msg.twist.twist.angular.z
            self.angular_velocities.append(omega)
    
    def run_calibration(self):
        print("\n" + "=" * 50)
        print(" AL-iLQR Go2 虚拟轴距标定工具")
        print("=" * 50)
        print(f"\n测试参数：")
        print(f"  线速度: {self.test_linear_vel} m/s")
        print(f"  角速度: {self.test_angular_vel} rad/s")
        print(f"  持续时间: {self.test_duration} s")
        print(f"  里程计话题: {self.odom_topic}")
        print(f"\n[WARNING] 请确保机器人周围有足够空间！")
        
        input("\n按 Enter 开始测试（Ctrl+C 取消）...")
        
        # 等待里程计
        print("\n[INFO] 等待里程计数据...")
        try:
            rospy.wait_for_message(self.odom_topic, Odometry, timeout=5.0)
        except rospy.ROSException:
            print(f"[ERROR] 无法收到 {self.odom_topic}，请检查 Odin 驱动是否运行")
            return
        
        # 预热 1 秒
        print("[INFO] 预热中...")
        cmd = Twist()
        cmd.linear.x = self.test_linear_vel
        cmd.angular.z = self.test_angular_vel
        
        rate = rospy.Rate(20)
        for _ in range(20):
            self.cmd_pub.publish(cmd)
            rate.sleep()
        
        # 开始记录
        print(f"[INFO] 开始记录（{self.test_duration}s）...")
        self.angular_velocities = []
        self.recording = True
        
        start_time = time.time()
        while time.time() - start_time < self.test_duration and not rospy.is_shutdown():
            self.cmd_pub.publish(cmd)
            rate.sleep()
        
        self.recording = False
        
        # 停止机器人
        stop_cmd = Twist()
        for _ in range(10):
            self.cmd_pub.publish(stop_cmd)
            rate.sleep()
        
        # 分析结果
        self.analyze_results()
    
    def analyze_results(self):
        if len(self.angular_velocities) < 10:
            print("[ERROR] 数据点不足，请检查里程计")
            return
        
        # 去掉前后 10% 的数据（过渡阶段）
        n = len(self.angular_velocities)
        trimmed = self.angular_velocities[int(n*0.1):int(n*0.9)]
        
        actual_omega = np.mean(trimmed)
        actual_omega_std = np.std(trimmed)
        expected_omega = self.test_angular_vel
        
        print("\n" + "=" * 50)
        print(" 标定结果")
        print("=" * 50)
        print(f"\n  数据点数: {len(trimmed)}")
        print(f"  期望角速度: {expected_omega:.4f} rad/s")
        print(f"  实际角速度: {actual_omega:.4f} ± {actual_omega_std:.4f} rad/s")
        print(f"  比值: {actual_omega / expected_omega:.4f}")
        
        # 计算建议的 virtual_wheelbase
        current_wheelbase = 0.4  # 默认值
        if abs(actual_omega) > 0.01:
            suggested_wheelbase = current_wheelbase * expected_omega / actual_omega
            print(f"\n  当前 virtual_wheelbase: {current_wheelbase}")
            print(f"  建议 virtual_wheelbase: {suggested_wheelbase:.4f}")
            print(f"\n  请在 config/al_ilqr_go2.yaml 中修改：")
            print(f"    control_mapping:")
            print(f"      virtual_wheelbase: {suggested_wheelbase:.4f}")
        else:
            print("\n[ERROR] 实际角速度接近零，可能：")
            print("  1. 机器人未真正运动")
            print("  2. 里程计数据异常")
            print("  3. unitree_control 未正常工作")
        
        print("\n" + "=" * 50)


if __name__ == '__main__':
    try:
        calibrator = WheelbaseCalibrator()
        calibrator.run_calibration()
    except rospy.ROSInterruptException:
        print("\n[INFO] 标定已取消")
    except KeyboardInterrupt:
        print("\n[INFO] 标定已取消")
