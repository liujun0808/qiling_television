#!/bin/bash

echo "=== 停止 XRoboToolkit PC端 ==="

echo "[1/2] 停止 RobotLinuxDemo..."
pkill -f RobotLinuxDemo.x86_64 2>/dev/null || true

sleep 1

echo "[2/2] 停止 RoboticsService..."
pkill -f RoboticsServiceProcess 2>/dev/null || true

sleep 1

echo
echo "=== 检查残留进程 ==="

DEMO_PID=$(pgrep -f RobotLinuxDemo.x86_64 || true)
SERVICE_PID=$(pgrep -f RoboticsServiceProcess || true)

if [ -n "$DEMO_PID" ]; then
    echo "警告：RobotLinuxDemo 仍然运行，PID: $DEMO_PID"
else
    echo "RobotLinuxDemo     ✅ 已停止"
fi

if [ -n "$SERVICE_PID" ]; then
    echo "警告：RoboticsService 仍然运行，PID: $SERVICE_PID"
else
    echo "RoboticsService    ✅ 已停止"
fi

echo
echo "=== XRoboToolkit PC端已停止 ==="