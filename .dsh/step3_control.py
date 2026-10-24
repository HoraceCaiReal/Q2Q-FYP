#!/usr/bin/env python3
"""Run in the Ubuntu VM after sourcing step3_env.sh. Own only our process group."""

import argparse
from datetime import datetime
import fcntl
import json
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

ROOT = Path.home() / '.q2q/runs'
STATE = ROOT / 'step3_control.json'
CLIENT = Path.home() / 'ros2_ws/src/robot_bringup/scripts/nav_acceptance.py'


def token(pid):
    try:
        fields = Path('/proc/%s/stat' % pid).read_text().rsplit(')', 1)[1].split()
        return None if fields[0] == 'Z' else fields[19]
    except FileNotFoundError:
        return None


def load():
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def owned(state):
    return bool(state and token(state['pid']) == state['start_tick'])


def marked(state):
    if not state:
        return []
    tag = ('Q2Q_STEP3_RUN=' + state['run_dir']).encode()
    result = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if tag in (entry / 'environ').read_bytes().split(b'\0'):
                result.append((int(entry.name), token(int(entry.name))))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass
    return result


def start(gui=False, world='q2q_warehouse', gripper_collision='mesh', position_gain=.1, gripper_mode='mimic', kinematic_carry=False):
    if owned(load()) or marked(load()):
        raise RuntimeError('Stack already running; use status/test/stop.')
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            command = (entry / 'cmdline').read_bytes().replace(b'\0', b' ')
            if command.startswith(b'ign gazebo ') and b'q2q_' in command:
                raise RuntimeError('Residual Q2Q Gazebo PID %s; stop it before starting.' % entry.name)
        except (FileNotFoundError, PermissionError):
            pass
    run = ROOT / ('step3_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    run.mkdir(parents=True)
    from ament_index_python.packages import get_package_share_directory
    world_source = Path(get_package_share_directory('robot_worlds'))/'worlds'/(world+'.sdf')
    snapshot = run/'world.sdf'
    snapshot.write_bytes(world_source.read_bytes())
    if kinematic_carry:
        if world != 'q2q_warehouse_grasp50':
            raise RuntimeError('Kinematic carry profile requires the explicit 50 mm fixture')
        tree = ET.parse(snapshot)
        tree.getroot().find('world/physics/max_step_size').text = '.02'
        tree.write(snapshot, encoding='utf-8', xml_declaration=True)
    command = ['ros2', 'launch', 'robot_bringup', 'bringup.launch.py',
               'gui:=true' if gui else 'gui:=false', 'world:='+str(snapshot), 'nav:=true', 'gz_verbose:=2']
    if gui:
        # 演示要看整个过程：Gazebo 3D 视口在这台机器上曾因软件 GL 空白，RViz 作为并行视图一起开
        command.append('rviz:=true')
    with (run / 'stack.log').open('w') as log:
        env = dict(os.environ, Q2Q_STEP3_RUN=str(run), Q2Q_GRIPPER_COLLISION=gripper_collision,
                   Q2Q_POSITION_GAIN=str(position_gain), Q2Q_GRIPPER_MODE=gripper_mode)
        if gui:
            env.setdefault('DISPLAY', ':0')  # Gazebo 窗口要 DISPLAY；SSH 会话里默认没有
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env,
                                   stdin=subprocess.DEVNULL, start_new_session=True)
    state = dict(pid=process.pid, start_tick=token(process.pid), run_dir=str(run),
                 command=command, middleware=os.environ.get('RMW_IMPLEMENTATION'),
                 dds_profile=os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE'),
                 kinematic_carry=kinematic_carry,
                 gripper_collision=gripper_collision, position_gain=position_gain, gripper_mode=gripper_mode)
    state.update(world=world, world_sha256=hashlib.sha256(snapshot.read_bytes()).hexdigest())
    STATE.write_text(json.dumps(state, indent=2))
    (run/'control.json').write_text(json.dumps(state, indent=2))
    print(json.dumps(state), flush=True)
    print('Started. Readiness is checked by test; start alone is not a PASS.')


def stop():
    state = load()
    members = marked(state)
    if owned(state) and not members:
        members = [(state['pid'], state['start_tick'])]
    if not members:
        print('No owned stack running.'); return
    for sig, limit in [(signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 3)]:
        live = [(pid, stamp) for pid, stamp in members if token(pid) == stamp]
        for pid, stamp in live:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + limit
        while any(token(pid) == stamp for pid, stamp in live) and time.monotonic() < deadline:
            time.sleep(0.2)
    if marked(state):
        raise RuntimeError('Owned stack did not stop; inspect run log, no broad pkill performed.')
    print('Owned stack stopped.')


def test():
    state = load()
    if not owned(state):
        raise RuntimeError('Start the stack first.')
    run = Path(state['run_dir']) / ('test_' + datetime.now().strftime('%H%M%S'))
    run.mkdir()
    results = []
    for name, goal, cancel in [('goal_a', [3, -2.5, 0], 0),
                               ('goal_b', [-3.5, 2, 3.14], 0),
                               ('cancel', [3.5, 4, 0], 8)]:
        command = ['python3', str(CLIENT), '--goal', *map(str, goal),
                   '--output', str(run / (name + '.json')),
                   '--cancel-after', str(cancel), '--ros-args', '-p', 'use_sim_time:=true',
                   '-r', '/tf:=/a200_0000/tf', '-r', '/tf_static:=/a200_0000/tf_static']
        print('TEST', name, flush=True)
        with (run / (name + '.log')).open('w') as log:
            process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True)
            for line in process.stdout:
                print(line, end='', flush=True); log.write(line); log.flush()
            code = process.wait()
        result = json.loads((run / (name + '.json')).read_text())
        results.append({k: v for k, v in result.items() if k != 'trace'})
        if code != 0 or not result['passed']:
            break
    passed = len(results) == 3 and all(r['passed'] for r in results)
    summary = dict(passed=passed, run_dir=str(run), results=results)
    (run / 'summary.json').write_text(json.dumps(summary, indent=2))
    print('STEP3_PASS=' + str(passed), 'ARTIFACTS=' + str(run), flush=True)
    return 0 if passed else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'stop', 'status', 'test'])
    parser.add_argument('--gui', action='store_true', help='带 Gazebo 原生窗口启动（默认 headless）')
    parser.add_argument('--kinematic-carry', action='store_true', help='Explicit 20ms kinematic carry simulation profile; not physical grasp validation')
    parser.add_argument('--world', choices=['q2q_warehouse', 'q2q_warehouse_grasp50', 'q2q_warehouse_grasp80'], default='q2q_warehouse')
    parser.add_argument('--gripper-collision', choices=['mesh', 'convex'], default='mesh')
    parser.add_argument('--position-gain', type=float, default=.1)
    parser.add_argument('--gripper-mode', choices=['mimic', 'commanded', 'suction'], default='mimic')
    args = parser.parse_args()
    ROOT.mkdir(parents=True, exist_ok=True)
    with (ROOT / 'step3_control.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.action == 'start':
            if not .05 <= args.position_gain <= 1.:
                parser.error('position gain must be within [.05,1]')
            start(args.gui, args.world, args.gripper_collision, args.position_gain, args.gripper_mode, args.kinematic_carry)
        elif args.action == 'stop':
            stop()
        elif args.action == 'test':
            return test()
        else:
            state = load()
            print(json.dumps(dict(running=owned(state), **state), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
