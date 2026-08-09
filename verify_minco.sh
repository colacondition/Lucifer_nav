#!/bin/bash
# MINCO 验证脚本

echo "=== MINCO Path Smoother 验证 ==="
echo ""

echo "1. 检查节点是否运行："
ros2 node list | grep minco
echo ""

echo "2. 检查话题连接："
echo "输入路径 (/plan_raw):"
ros2 topic info /plan_raw | grep -E "Publisher|Subscription"
echo ""
echo "输出路径 (/plan):"
ros2 topic info /plan | grep -E "Publisher|Subscription"
echo ""

echo "3. 检查参数配置："
ros2 param get /rm_minco_path_smoother smooth_weight
ros2 param get /rm_minco_path_smoother obstacle_weight
ros2 param get /rm_minco_path_smoother data_weight
echo ""

echo "4. 给个目标点，然后检查路径点数："
echo "   /plan_raw 原始路径点数 vs /plan MINCO 输出点数"
echo "   路径应该平滑但保持原形状，不会掰成直线"
echo ""

echo "=== 如果上面都正常，MINCO 集成成功 ==="
