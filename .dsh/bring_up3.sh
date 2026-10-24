#!/usr/bin/env bash
# Start once; inspect readiness. No global kills or shared-memory deletion.
set -eo pipefail
source /home/ros/.q2q/step3_env.sh
if ! python3 - <<'PY'
import json
from pathlib import Path
try:
    s=json.loads((Path.home()/'.q2q/runs/step3_control.json').read_text())
    f=Path('/proc/%s/stat'%s['pid']).read_text().rsplit(')',1)[1].split()
    raise SystemExit(0 if f[0]!='Z' and f[19]==s['start_tick'] else 1)
except (FileNotFoundError, KeyError):
    raise SystemExit(1)
PY
then
  python3 /home/ros/.q2q/step3_control.py start --world q2q_warehouse_grasp50 --kinematic-carry
fi
python3 - <<'PY'
import time,rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
from controller_manager_msgs.srv import ListControllers
rclpy.init()
n=Node('step7_stack_readiness')
cm=n.create_client(ListControllers,'/a200_0000/controller_manager/list_controllers')
nav=n.create_client(GetState,'/a200_0000/bt_navigator/get_state')
required={'joint_state_broadcaster','platform_velocity_controller',
          'arm_0_joint_trajectory_controller','arm_0_gripper_controller'}
deadline=time.monotonic()+180
ready=False
while time.monotonic()<deadline and not ready:
    if cm.service_is_ready() and nav.service_is_ready():
        a,b=cm.call_async(ListControllers.Request()),nav.call_async(GetState.Request())
        rclpy.spin_until_future_complete(n,a,timeout_sec=3)
        rclpy.spin_until_future_complete(n,b,timeout_sec=3)
        if a.done() and b.done() and not a.exception() and not b.exception():
            active={c.name for c in a.result().controller if c.state=='active'}
            ready=required<=active and b.result().current_state.id==3
        if not a.done(): a.cancel()
        if not b.done(): b.cancel()
    rclpy.spin_once(n,timeout_sec=.2)
n.destroy_node()
rclpy.shutdown()
print('READY' if ready else 'NOT_READY: inspect stack.log; no automatic restart performed')
raise SystemExit(0 if ready else 1)
PY
