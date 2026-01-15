#!/bin/bash
# quick_fix_ros2.sh - 快速修复脚本

echo "快速修复ROS 2共享内存错误..."

# 停止守护进程
ros2 daemon stop 2>/dev/null || echo "停止守护进程失败或未运行"

# 清理共享内存
sudo rm -f /dev/shm/fastrtps_port* 2>/dev/null
sudo rm -f /dev/shm/ros2_daemon* 2>/dev/null

# 重启守护进程
ros2 daemon start 2>/dev/null || echo "启动守护进程失败"

echo "快速修复完成！"