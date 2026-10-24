#!/usr/bin/env python3
"""第 7 步整链一次真跑：起 5 个节点 → 发一条 START → 盯到终态。

为什么要有这个入口：第 7 步的节点是**五个进程**（move_group / suction_relay /
grasp_executor / state_recorder / orchestration_node），手敲五条命令 + 一条 topic pub
既不可复现、也容易把上一轮残留混进来。这里把它们绑成一次「一幕」，
日志全部落进本轮 run_dir，结束时按进程组回收，不留孤儿。

用法（VM 内，已 source ~/.q2q/step3_env.sh，栈已 Start）：
    python3 ~/.q2q/step7_trial.py --arrive-x 0.80 --freshness 1.5
"""

import argparse
import atexit
import fcntl
import json
import os
import signal
import subprocess
import time
import sys
from datetime import datetime
from pathlib import Path

import rclpy
from q2q_interfaces.msg import TaskCommand, TaskState
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import Bool

HOME = Path.home()
RUNS = HOME / '.q2q/runs'
CONTROL = RUNS / 'step3_control.json'
TERMINAL = ('DONE', 'FAILED')


def stack_dir():
    state = json.loads(CONTROL.read_text())
    fields = Path('/proc/%s/stat' % state['pid']).read_text().rsplit(')', 1)[1].split()
    if fields[0] == 'Z' or fields[19] != state['start_tick']:
        raise SystemExit('Stack is not running; refusing stale run directory')
    return Path(state['run_dir'])


def launch(name, command, log_dir, env):
    log = (log_dir / (name + '.log')).open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                               stdin=subprocess.DEVNULL, start_new_session=True, env=env)
    print('START %-10s pid=%d  %s' % (name, process.pid, ' '.join(command)), flush=True)
    return process


def ros_args(params):
    """dict -> --ros-args -p use_sim_time:=true -p k:=v ..."""
    out = ['--ros-args', '-p', 'use_sim_time:=true']
    for key, value in params.items():
        out += ['-p', '%s:=%s' % (key, value)]
    return out


class Watcher(Node):
    def __init__(self, episode, target, timeout_s, nav_timeout, task_id, cancel_at=None):
        super().__init__('step7_trial')
        self.episode, self.task_id = episode, task_id
        self.target, self.timeout_s, self.nav_timeout = target, timeout_s, nav_timeout
        self.started_at = time.monotonic()
        self.cancel_at, self.cancel_sent = cancel_at, False
        self.sent, self.done, self.code = False, False, 0
        self.last_state, self.final, self.transitions = None, None, []
        self.carry_active = None
        self.create_subscription(Bool, '/q2q/low/suction_active', self.on_carry, 1)
        self.pub = self.create_publisher(TaskCommand, '/q2q/mid/task_command', 10)
        self.create_subscription(TaskState, '/q2q/mid/task_state', self.on_state, 20)
        self.create_timer(0.5, self.step, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('trial 就绪：episode=%s target=%.2f' % (episode, target[0]))

    def on_state(self, msg):
        if not self.sent:
            self.send_start()                     # 状态话题一通就发命令；就绪由闸门兜
        if self.cancel_at == msg.state and not self.cancel_sent:
            command = TaskCommand(command=TaskCommand.CANCEL, request_id=self.episode+'_cancel')
            self.pub.publish(command)
            self.cancel_sent = True
            print('SENT CANCEL at '+msg.state, flush=True)
        if msg.state != self.last_state:
            self.last_state = msg.state
            self.transitions.append(dict(state=msg.state, generation=msg.generation,
                                         x=round(msg.base_x, 4), nav=msg.nav_status,
                                         locked=msg.base_command_locked, blocked_by=msg.blocked_by))
            print('TRANS %3d  %-12s x=%.3f nav=%-9s locked=%s  blocked_by=%s'
                  % (msg.generation, msg.state, msg.base_x, msg.nav_status,
                     msg.base_command_locked, msg.blocked_by), flush=True)
        canceled = (self.cancel_sent and msg.state == 'IDLE' and msg.base_fresh
                    and msg.base_stopped and not msg.base_command_locked)
        if (msg.state in TERMINAL or canceled) and self.final is None:
            self.final = dict(state=msg.state, blocked_by=msg.blocked_by, x=msg.base_x,
                              late_events=msg.late_events, generation=msg.generation,
                              base_command_locked=msg.base_command_locked)
            self.final['canceled'] = canceled

    def on_carry(self, msg):
        self.carry_active = msg.data

    def send_start(self):
        command = TaskCommand()
        command.header.frame_id = 'map'
        command.request_id = self.episode
        command.command = TaskCommand.START
        command.task_id = self.task_id
        command.target_x, command.target_y, command.target_yaw = self.target
        command.timeout_s = self.nav_timeout
        self.pub.publish(command)
        self.sent = True
        print('SENT START request_id=%s target=(%.2f, %.2f, %.2f)' % (self.episode, *self.target),
              flush=True)

    def step(self):
        now = time.monotonic()
        if self.final is not None:
            self.done, self.code = True, 0 if (self.final['state'] == 'DONE' or self.final['canceled']) else 1
        elif now - self.started_at > self.timeout_s:
            print('TRIAL_TIMEOUT after %.0f s' % self.timeout_s, flush=True)
            self.done, self.code = True, 1
        elif not self.sent and now - self.started_at > 120.0:
            print('NO task_state in 120 s：orchestration_node 没起来', flush=True)
            self.done, self.code = True, 2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target-x', type=float, default=0.85)
    parser.add_argument('--target-y', type=float, default=0.0)
    parser.add_argument('--target-yaw', type=float, default=0.0)
    parser.add_argument('--arrive-x', type=float, default=0.80)
    parser.add_argument('--nav-timeout', type=float, default=300.0)
    parser.add_argument('--dock-timeout', type=float, default=110.0)
    parser.add_argument('--wall-scale', type=float, default=6.0)
    parser.add_argument('--trial-timeout', type=float, default=1500.0)
    parser.add_argument('--grasp-timeout', type=float, default=900.0)
    parser.add_argument('--freshness', type=float, default=1.5)
    parser.add_argument('--task-id', default='step7-full')
    parser.add_argument('--no-move-group', action='store_true')
    parser.add_argument('--cancel-at', choices=['NAV_TO_PICK', 'PICKING', 'STOWING', 'NAV_TO_PLACE'])
    args = parser.parse_args()

    # Never kill another trial to take ownership of its robot.
    lock_file = (RUNS / 'step7.lock').open('a')
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('Another step7 trial owns the stack; refusing duplicate execution')

    directory = stack_dir()
    stack_profile = json.loads(CONTROL.read_text())
    episode = 'step7_' + datetime.now().strftime('%m%d_%H%M%S')
    env = dict(os.environ, Q2Q_STEP3_RUN=str(directory), Q2Q_WALL_SCALE=str(args.wall_scale))
    world = str(directory / 'world.sdf')
    directory = directory / episode
    directory.mkdir()

    children = []
    def cleanup():
        for sig, delay in ((signal.SIGINT, 3), (signal.SIGKILL, 0)):
            for process in children:
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    pass
            if delay:
                time.sleep(delay)
    atexit.register(cleanup)
    def terminate(signum, frame):
        raise SystemExit(128+signum)
    signal.signal(signal.SIGTERM, terminate)
    if not args.no_move_group:
        children.append(launch('move_group', ['ros2', 'launch', 'robot_moveit', 'move_group.launch.py'],
                               directory, env))
    # A fresh move_group has an empty scene. Populate it before allowing START.
    scene_script = HOME / 'ros2_ws/src/robot_moveit/scripts/scene_loader.py'
    with (directory / 'scene.log').open('w') as log:
        loaded = subprocess.run(['python3', str(scene_script), '--world', world],
                                stdout=log, stderr=subprocess.STDOUT, env=env, timeout=130)
    if loaded.returncode:
        for process in children:
            os.killpg(process.pid, signal.SIGINT)
        raise SystemExit('Planning scene load failed; START not sent (see scene.log)')
    children.append(launch('relay', ['ros2', 'run', 'q2q_low', 'suction_relay'] + ros_args({})
                           + ['-r', '/tf:=/a200_0000/tf', '-r', '/tf_static:=/a200_0000/tf_static'],
                           directory, env))
    # 本轮走运动学搬运（grasp_mode=suction_approx）；真夹持还没成立，不在这里假装
    children.append(launch('exec', ['ros2', 'run', 'q2q_low', 'grasp_executor']
                           + ros_args(dict(use_suction='true', world_sdf=world,
                                           grasp_timeout_s=args.grasp_timeout)), directory, env))
    children.append(launch('rec', ['ros2', 'run', 'q2q_record', 'state_recorder'], directory, env))
    children.append(launch('orch', ['ros2', 'run', 'q2q_orchestration', 'orchestration_node']
                           + ros_args(dict(episode_id=episode, task_id=args.task_id,
                                           arrive_x=args.arrive_x, nav_ready_timeout_s=45.0,
                                           freshness_window=args.freshness,
                                           dock_timeout_s=args.dock_timeout,
                                           grasp_timeout_s=args.grasp_timeout)),
                           directory, env))

    rclpy.init()
    watcher = Watcher(episode, (args.target_x, args.target_y, args.target_yaw),
                      args.trial_timeout, args.nav_timeout, args.task_id, args.cancel_at)
    while rclpy.ok() and not watcher.done:
        rclpy.spin_once(watcher, timeout_sec=0.2)
    summary = dict(episode=episode, exit_code=watcher.code, final=watcher.final,
                   stack_profile=stack_profile, grasp_mode='suction_approx',
                   transitions=watcher.transitions,
                   record=str(HOME / '.q2q/records' / episode / 'task_state.jsonl'))
    if watcher.final and watcher.final['state'] == 'DONE':
        sys.path.insert(0, str(HOME/'ros2_ws/src/robot_bringup/scripts'))
        import object_pose
        import math
        deadline, previous, stable = time.monotonic()+30., None, 0
        while time.monotonic() < deadline:
            physical = object_pose.read_entities()
            box = physical['pick_bin']['position']
            base = physical['a200_0000/robot']
            yaw = base['yaw']
            expected = (base['position'][0]+.4*math.cos(yaw),
                        base['position'][1]+.4*math.sin(yaw))
            error = math.hypot(box[0]-expected[0], box[1]-expected[1])
            resting = (error < .08 and .015 < box[2] < .04 and previous is not None
                       and math.dist(box, previous) < .005)
            stable = stable+1 if resting else 0
            previous = box
            summary['delivery'] = dict(box_world=box, expected_xy=expected,
                                       xy_error_m=error, stable_samples=stable)
            if stable >= 3:
                break
            time.sleep(.3)
        summary['passed'] = (stable >= 3 and not watcher.final['base_command_locked']
                             and watcher.final['late_events'] == 0)
        if not summary['passed']:
            summary['exit_code'] = watcher.code = 1
    elif watcher.final and watcher.final.get('canceled'):
        summary['passed'] = watcher.final['late_events'] == 0
    else:
        summary['passed'] = False
    (directory / ('step7_%s.json' % episode)).write_text(json.dumps(summary, indent=2,
                                                                    ensure_ascii=False))
    watcher.destroy_node()
    rclpy.shutdown()
    cleanup()
    atexit.unregister(cleanup)
    print('SUMMARY ' + json.dumps(dict(episode=episode, exit_code=watcher.code,
                                       final=watcher.final, transitions=len(watcher.transitions))),
          flush=True)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
