#!/bin/bash

pkill -f RoboticsServiceProcess 2>/dev/null || true
pkill -f RobotLinuxDemo.x86_64 2>/dev/null || true

env \
-u http_proxy \
-u https_proxy \
-u all_proxy \
-u HTTP_PROXY \
-u HTTPS_PROXY \
-u ALL_PROXY \
NO_PROXY="localhost,127.0.0.1,192.168.110.0/24" \
no_proxy="localhost,127.0.0.1,192.168.110.0/24" \
bash /opt/apps/roboticsservice/runService.sh

sleep 2

cd /opt/apps/roboticsservice/SDKDemo/RobotUnityDemo

env \
-u DISPLAY \
-u WAYLAND_DISPLAY \
-u http_proxy \
-u https_proxy \
-u all_proxy \
-u HTTP_PROXY \
-u HTTPS_PROXY \
-u ALL_PROXY \
NO_PROXY="localhost,127.0.0.1,192.168.110.0/24" \
no_proxy="localhost,127.0.0.1,192.168.110.0/24" \
LD_LIBRARY_PATH="/opt/apps/roboticsservice:/opt/apps/roboticsservice/lib:/opt/apps/roboticsservice/SDK/x64:$LD_LIBRARY_PATH" \
./RobotLinuxDemo.x86_64 \
-batchmode \
-nographics \
-logFile /home/ub/robotlinuxdemo_noproxy.log \
>/home/ub/robotlinuxdemo_noproxy_stdout.log 2>&1 &

echo "XRoboToolkit PC端已启动"