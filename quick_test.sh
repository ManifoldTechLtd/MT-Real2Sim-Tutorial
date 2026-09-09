#!/bin/bash
# 快速测试 - 验证三类机器人是否能正常启动

echo "======================================"
echo "Isaac Navigation Sim - 快速测试"
echo "======================================"
echo ""
echo "本脚本将依次测试三类机器人的启动"
echo "每个测试持续 30 秒，然后自动停止"
echo ""
read -p "按 Enter 开始测试..."

ROBOTS=("kinematic_car" "spot" "go2")

for robot in "${ROBOTS[@]}"; do
    echo ""
    echo "======================================"
    echo "测试: $robot"
    echo "======================================"

    # 启动
    echo "启动 $robot (30秒后自动停止)..."
    timeout 30 ./start.sh --robot "$robot" || true

    # 停止
    echo "停止容器..."
    ./start.sh --stop

    echo "✓ $robot 测试完成"
    sleep 2
done

echo ""
echo "======================================"
echo "所有测试完成！"
echo "======================================"
echo ""
echo "如果所有机器人都能启动，说明系统正常"
echo "可以开始正式使用了"
