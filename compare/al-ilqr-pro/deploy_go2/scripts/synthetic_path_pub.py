#!/usr/bin/env python3
# Phase 3.4 辅助：向 /initial_path 注入一条合成的直线全局路径，
# 用于在 map_planner "Start/goal occupied" 无法出图时，单独验证
# al_ilqr 求解器本体（是否收敛/发布 optimized_path/cmd_vel 限幅/求解频率）。
# 路径在 map 系，从机器人当前位姿沿其朝向向前 ~5m。
import rospy, math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# 默认在 odom 系、机器人状态附近(0,0)沿 +x 向前，可用 rosparam 覆盖
X0 = float(__import__("os").environ.get("SP_X0", "0.0"))
Y0 = float(__import__("os").environ.get("SP_Y0", "0.0"))
YAW = float(__import__("os").environ.get("SP_YAW", "0.0"))
N, STEP = 30, 0.2   # 30 点，间隔 0.2m -> 约 5.8m
FRAME = __import__("os").environ.get("SP_FRAME", "odom")

def build():
    p = Path()
    p.header.frame_id = FRAME
    p.header.stamp = rospy.Time.now()
    dx, dy = math.cos(YAW), math.sin(YAW)
    for i in range(N):
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.header.stamp = p.header.stamp
        ps.pose.position.x = X0 + i * STEP * dx
        ps.pose.position.y = Y0 + i * STEP * dy
        ps.pose.orientation.z = math.sin(YAW / 2.0)
        ps.pose.orientation.w = math.cos(YAW / 2.0)
        p.poses.append(ps)
    return p

if __name__ == "__main__":
    rospy.init_node("synthetic_path_pub")
    pub = rospy.Publisher("/initial_path", Path, queue_size=1, latch=True)
    r = rospy.Rate(10)
    rospy.loginfo("publishing synthetic /initial_path frame=%s N=%d", FRAME, N)
    while not rospy.is_shutdown():
        pub.publish(build())
        r.sleep()
