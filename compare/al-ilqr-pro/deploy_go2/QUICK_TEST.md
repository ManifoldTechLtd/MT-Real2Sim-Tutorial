# AL-iLQR Go2 傻瓜式测试手册（QUICK_TEST.md）

> 每个终端都是在**你自己的电脑**上打开的，通过 SSH 连到 Go2 NX。
> 按顺序开终端，每个终端把整段命令原样复制粘贴回车即可。
> ⚠️ 只要 odin driver 一启动，底盘就是通的（host_sdk_sample 直连 /cmd_vel），
>    发目标点后机器人会**真实运动**。测试前确保遥控器在手（L1+L2 硬急停）。

---

## 准备（每次测试前做一次）

- Go2 开机，趴下→站立正常
- 机器人放在走廊空旷区（离墙 ≥ 1m）
- 网线连接正常（能 ping 通 192.168.123.18）

---

## 终端 1：启动 Odin 驱动（定位/里程计/点云）

```bash
sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.123.18
# 登录后执行：
source /opt/ros/noetic/setup.bash
source ~/k02_nav/ros_ws_base_inspection/ros_ws/devel/setup.bash
roslaunch odin_ros_driver odin1_ros1.launch
```

等待日志出现 **`relocalization success!`**（约 30 秒）。
如果一直不出现：用遥控器操纵机器人慢走 2~3 米，帮助重定位。

---

## 终端 2：启动导航栈（地图/全局规划/激光）

```bash
sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.123.18
# 登录后执行：
source /opt/ros/noetic/setup.bash
source ~/code_ilqr/devel/setup.bash
roslaunch map_planner whole.launch use_unitree_control:=false
```

没有报红字（ERROR）即可继续。

---

## 终端 3：启动 AL-iLQR 局部规划器

```bash
sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.123.18
# 登录后执行：
source /opt/ros/noetic/setup.bash
source ~/code_ilqr/devel/setup.bash
roslaunch al_ilqr_go2 al_ilqr_go2.launch
```

看到 `AL-iLQR Planner node started.` 即成功。

---

## 终端 4：检查状态 + 发目标点（机器人会动！）

> 💡 如需录制数据包分析问题，先去看“终端 5”启动录制，再回来发目标点。

```bash
sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.123.18
# 登录后执行：
source /opt/ros/noetic/setup.bash
source ~/code_ilqr/devel/setup.bash
```

### 4.1 先看状态（应显示 WAITING_FOR_PATH）

```bash
rostopic echo -n3 /al_ilqr/status
```

### 4.2 发目标点（⚠️ 发完机器人立刻开始走）

```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: -20.0, y: 3.8, z: 0.0}, orientation: {z: 0.690, w: 0.723}}}' --once
```

> 把 x/y 换成你想去的地图坐标即可。目标点必须在地图空闲区，
> 否则终端 2 会报 "Start or goal is occupied"。

### 4.3 观察运行状态（正常样例）

```bash
rostopic echo /al_ilqr/status
```

正常应类似：
```
OK | cvg=1 | iter=(1/3) | 0.9ms | analytical | viol=0.005 | obs=5 | min_r=1.53 | a=1.005 | s=0.190 | v=0.618 | w=0.298
```
- `cvg=1`：求解收敛
- `v=0.5~0.6`：正常前进速度
- 到达目标后自动停车，日志出现 `goal reached`

---

## 终端 5：录制数据包 bag（出问题时用于分析，建议每次测试都录）

> 在**发目标点之前**启动录制，测完按 Ctrl+C 停止。

```bash
sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.123.18
# 登录后执行：
source /opt/ros/noetic/setup.bash
bash ~/code_ilqr/src/al_ilqr_go2/scripts/record_bag.sh corridor
```

- `corridor` 是场景名，自己改（如 turn、dyn_obs），只影响文件名
- 限时录制：`bash ... record_bag.sh corridor 120`（120 秒自动停）
- 深度排查需要原始点云时：`bash ... record_bag.sh corridor 60 --with-cloud`（体积大）
- bag 保存在 NX 的 `~/bags/` 目录，文件名带时间戳和场景名

### 测完后把 bag 拷回开发机（开发机执行）

```bash
# 先看有哪些 bag
sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.123.18 'ls -lh ~/bags/'

# 拷最新一个（把文件名换成上面看到的）
mkdir -p ~/go2_bags && sshpass -p 123 scp 'unitree@192.168.123.18:~/bags/文件名.bag' ~/go2_bags/
```

### 出问题时交给开发分析的三样东西

1. bag 文件（上面拷回来的）
2. 现象描述（机器人做了什么/没做什么，大概几点几分发生）
3. 终端 3（al_ilqr）屏幕上的报错/告警截图

---

## 🛑 急停（任选其一，从快到慢）

1. **遥控器 L1+L2**（最快，物理急停）
2. 任意已登录终端执行：
   ```bash
   rosnode kill /al_ilqr_go2_node
   timeout 3 rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}}' -r 20
   ```

---

## 测试结束：关闭所有程序

任意已登录 NX 的终端执行（或直接在各终端按 Ctrl+C）：

```bash
pkill -f '[r]oslaunch'; pkill -f '[r]oscore'; pkill -f '[r]osmaster'; pkill -f '[h]ost_sdk_sample'; sleep 2; pkill -9 -f '[r]osout'
# 验证（输出 0 即全部关闭）：
ps aux | grep -cE '[r]oslaunch|[r]osmaster|[h]ost_sdk|[a]l_ilqr|[m]ap_planner'
```

---

## 常见问题速查

| 现象 | 处理 |
|------|------|
| 终端 2 报 "Start or goal is occupied" | 机器人在地图膨胀区，遥控移到走廊正中间再重发目标点 |
| status 显示 WAITING_FOR_ODOM | 终端 1 的 driver 没起来或还没重定位成功 |
| status 显示 PATH_TF_FAILED | map→odom TF 缺失，重启终端 1 等 relocalization success |
| status 显示 SOLVER_FAILED 且频繁 | 记录现场日志（/al_ilqr/status 与障碍数 obs），交给开发排查 |
| 机器人抽提/停滞 | 看日志有无 "clearing warm start"，若持续请急停并记录 |
| 录制时提示磁盘不足 | NX 上清理旧 bag：`ls -lh ~/bags/` 后手动删除不要的 |

---

## 改代码后的重新部署（开发机执行）

```bash
# 1. 同步代码到 NX
sshpass -p 123 rsync -az --delete --exclude='.git' \
  /home/yu/article_and_warehouse/algo-warehouse/compare/al-ilqr-pro/deploy_go2/ \
  unitree@192.168.123.18:/home/unitree/code_ilqr/src/al_ilqr_go2/

# 2. NX 上编译
sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.123.18 \
  "bash -lc 'source /opt/ros/noetic/setup.bash && cd ~/code_ilqr && catkin build al_ilqr_go2 --cmake-args -DCMAKE_BUILD_TYPE=Release'"
```
