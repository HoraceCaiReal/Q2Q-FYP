#!/usr/bin/env bash
# 一键演示（VM 侧）：headless 起栈 -> 挂 Gazebo 窗口 -> 挂 RViz -> 跑完整整链。
# 设计原则：渲染与仿真分开进程。窗口崩了、被关掉、重开，都不影响正在跑的任务。
# 主机侧 .dsh/demo.ps1 会调用本脚本，并在关键状态自动截屏。
# 注意：不要用 set -u，ROS 的 setup.bash 里有未定义变量。
source /opt/ros/humble/setup.bash
source /home/ros/ros2_ws/install/setup.bash 2>/dev/null
source /home/ros/.q2q/step3_env.sh 2>/dev/null
export DISPLAY=:0
export XAUTHORITY=/home/ros/.Xauthority
export Q2Q_WALL_SCALE="${Q2Q_WALL_SCALE:-3}"

echo "[demo] 0/4 stop old stack and clean residual"
python3 /home/ros/.q2q/step3_control.py stop >/dev/null 2>&1
pkill -9 -f "ign gazebo" 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null
sleep 3
rm -rf /dev/shm/* 2>/dev/null

echo "[demo] 1/4 start stack headless"
python3 /home/ros/.q2q/step3_control.py start --world q2q_warehouse_grasp50 --kinematic-carry | tail -1
RUN=$(python3 -c 'import json,pathlib;print(json.loads((pathlib.Path.home()/".q2q/runs/step3_control.json").read_text())["run_dir"])')
echo "RUN=$RUN"

echo "[demo] 2/4 wait for controllers and nav action"
ready=0
for i in $(seq 1 36); do
  sleep 5
  timeout 25 ros2 control list_controllers -c /a200_0000/controller_manager > /tmp/lc.txt 2>/dev/null
  timeout 20 ros2 action list > /tmp/al.txt 2>/dev/null
  n=$(grep -c active /tmp/lc.txt 2>/dev/null)
  v=$(grep -c navigate_to_pose /tmp/al.txt 2>/dev/null)
  echo "        t=$((i*5))s active=${n:-0} nav=${v:-0}"
  if [ "${n:-0}" -ge 4 ] && [ "${v:-0}" -ge 1 ]; then ready=1; break; fi
done
if [ "$ready" -ne 1 ]; then
  echo "[demo] READY_FAILED - see $RUN/stack.log"
  exit 1
fi

echo "[demo] 3/4 attach Gazebo GUI client (ogre engine) and RViz"
nohup ign gazebo -g --gui-config /home/ros/.q2q/setup/a200_kinova/gui.config \
      --render-engine ogre > /tmp/gz_gui.log 2>&1 &
sleep 4
nohup ros2 run rviz2 rviz2 -d /opt/ros/humble/share/clearpath_viz/rviz/nav2.rviz \
      --ros-args -r /tf:=/a200_0000/tf -r /tf_static:=/a200_0000/tf_static > /tmp/rviz_demo.log 2>&1 &
sleep 3

echo "[demo] 4/4 run the full chain (one START)"
nohup bash /home/ros/.q2q/launch_trial.sh > /tmp/trial_demo.log 2>&1 &
sleep 3
LOG=$(ls -t "$RUN"/step7_console_*.log 2>/dev/null | head -1)
echo "DEMO_LAUNCHED run=$RUN"
echo "TRIAL_LOG=$LOG"
