#!/usr/bin/env python3
"""grasp_run：一次「真值闭环」的抓取尝试 —— 第 6 步唯一抓取入口。

为什么不用 SDF 相机：`virtual_camera` 画的是世界文件里的 `<pose>`，箱子被碰动后它看不见，
于是「同一套命令一次夹住、一次夹空」（10-23 复盘第 9 节）。本脚本的位姿全部来自
`/world/q2q_warehouse/pose/info`，指尖位置是**量的**真值、不是 FK 推的（MoveIt 看不到 mimic）。

判据见 docs/decisions/2026-10-23-第6步确定性抓取判据.md；失败立即中止，JSON 保留证据。

前提：车已停稳在工位，MoveIt 已加载干净场景，目标与桌子仍在碰撞场景。
    python3 grasp_run.py --steps 3
"""

import argparse
import json
import hashlib
import time
import signal
from pathlib import Path

import numpy as np

import grasp_io
import grasp_pose
import rclpy
from rclpy.node import Node

BOX_SDF = (1.20, 0.0, 0.54)   # 世界文件里的初始位姿（10-23 晚箱子降到台面），只用来判「基线干净」


APPROACHES = {'forward': np.array([1.0, 0.0, 0.0]), 'topdown': np.array([0.0, 0.0, -1.0])}
TCP_NOMINAL = 0.114   # 末端 -> 两指尖中点，10-23 实测；只用于第一次预抓取，之后按当前姿态重量


def main():
    def interrupted(signum, frame):
        raise KeyboardInterrupt('grasp process interrupted')
    parser = argparse.ArgumentParser()
    parser.add_argument('--approach', choices=sorted(APPROACHES), default='topdown',
                        help='接近方向：topdown 从上方下来（10-23 复盘：水平推进会把手张开的手指当刮板）')
    parser.add_argument('--open-grip', type=float, default=0.0)     # 0.0 = 张开（实测反直觉）
    parser.add_argument('--close-grip', type=float, default=0.15,
                        help='Shallow position closure for this 80 mm cube; .90 is EMPTY-gripper regression only')
    parser.add_argument('--effort', type=float, default=5.0, help='API compatibility only; position simulation does not enforce force')
    parser.add_argument('--pregrasp', type=float, default=0.05, help='爪心离箱心多远起手（沿接近方向）')
    parser.add_argument('--tip-past', type=float, default=-0.015,
                        help='Distal-link origins stay 15 mm above box centre: fingertips extend below origins')
    parser.add_argument('--steps', type=int, default=3)
    parser.add_argument('--nudge', type=float, default=0.008,
                        help='对准时一次最多挪多少（E3 实测：一次挪 25 mm 规划不出来）')
    parser.add_argument('--step', type=float, default=0.025, help='每步抬升高度（大步会甩掉箱子）')
    parser.add_argument('--output')
    parser.add_argument('--close-servo', action='store_true',
                        help='Increment closure while compensating measured TCP drift; never retry a failed action')
    parser.add_argument('--close-mode', choices=('contact', 'direct'), default='direct',
                        help='direct=一次性合到目标（已验收 P0–P7 的路径）；'
                             'contact=合到 stall 再留预紧——10-24 三次尝试均未通过，见 P6b 记录')
    parser.add_argument('--close-step', type=float, default=0.008, help='（contact 模式保留）探测步长')
    parser.add_argument('--contact-mm', type=float, default=0.2, help='（contact 模式保留）开口变化阈值')
    parser.add_argument('--contact-flat', type=int, default=2, help='（contact 模式保留）连续无进展步数')
    parser.add_argument('--preload', type=float, default=0.02,
                        help='（contact 模式保留）接触后再压的增量；实测 0.004 太弱、0.02 反而被 facade 走反')
    parser.add_argument('--hold', type=float, default=0.0,
                        help='抬升后静置多少秒再判 P6b。默认 0 = 保持已验收口径；>0 用于夹持持久性实验')
    parser.add_argument('--box-center', nargs=3, type=float, default=BOX_SDF,
                        help='Explicit world baseline; 50mm fixture uses 1.20 0 .525')
    parser.add_argument('--suction', action='store_true',
                        help='吸附式夹持（真空吸近似）：合爪后把物体与末端刚性连接。'
                             '必须写进记录 grasp_mode=suction_approx，不能当摩擦抓取')
    args = parser.parse_args()
    if args.steps < 3 or not 0 < args.step <= .025 or not 0 < args.nudge <= .008:
        parser.error('Require >=3 lifts, 0<step<=.025, 0<nudge<=.008')

    rclpy.init()
    signal.signal(signal.SIGTERM, interrupted)
    node = Node('grasp_run')
    report = dict(passed=False, approach=args.approach,
                  grasp_mode='suction_approx' if args.suction else 'friction')
    report['parameters'] = vars(args)
    description = Path.home()/'.q2q/setup/a200_kinova/robot.patched.urdf'
    if description.exists():
        model = description.read_bytes()
        report['model_sha256'] = hashlib.sha256(model).hexdigest()
        report['gripper_collision'] = 'convex' if b'collision_hulls/' in model else 'mesh'
    try:
        return run(node, args, report)
    except BaseException as error:
        report['error'] = str(error)
        if args.suction:
            # 异常路径必须松开，不允许把物体「挂」在手上退出
            try:
                report['suction_detach'] = grasp_io.suction(node, False)
            except Exception as suction_error:
                report['suction_error'] = str(suction_error)
        try:
            physical = grasp_io.read()
            report['final_truth'] = {name: physical[name] for name in
                [grasp_io.ROBOT, 'pick_bin', grasp_io.EE, grasp_io.LEFT, grasp_io.RIGHT]}
        except Exception as truth_error:
            report['truth_error'] = str(truth_error)
        return 2
    finally:
        report['motion_stop_confirmed'] = getattr(node, '_motion_stop_confirmed', True)
        if args.output:
            Path(args.output).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps(report, ensure_ascii=False, indent=1), flush=True)
        node.destroy_node()
        rclpy.shutdown()


def run(node, args, report):
    pair = grasp_io.clients(node)
    report['baseline_center_world'] = list(args.box_center)
    if pair is None:
        print(json.dumps({'error': 'move_action 或 gripper_cmd 不在'}, ensure_ascii=False))
        return 1
    move, grip = pair
    approach = APPROACHES[args.approach]
    quaternion = grasp_pose.approach_quaternion(approach, True)

    scene = grasp_io.read()
    robot, box = scene[grasp_io.ROBOT], scene['pick_bin']
    box_base = grasp_io.model_base(scene, 'pick_bin')
    report['P1'] = {'base_x': round(robot['position'][0], 4),
                    'active_controllers': grasp_io.active_controllers(node),
                    'yaw_deg': round(np.degrees(robot['yaw']), 2),
                    'box_vs_sdf_mm': round(1000 * float(np.linalg.norm(
                        np.subtract(box['position'], args.box_center))), 1)}
    report['P3'] = {'source': 'pose/info 真值',
                    'box_center_base': [round(float(v), 4) for v in box_base]}
    if report['P1']['box_vs_sdf_mm'] > 5:
        raise RuntimeError('P1: object is not at clean baseline; restart, do not chase it')
    later = grasp_io.read()[grasp_io.ROBOT]
    report['P2'] = dict(base_motion_mm=1000*float(np.linalg.norm(
        np.subtract(later['position'], robot['position']))),
        station_x=robot['position'][0], station_y=robot['position'][1])
    if (not .86 <= robot['position'][0] <= .90 or abs(robot['position'][1]) > .02
            or abs(robot['yaw']) > .04 or report['P2']['base_motion_mm'] > 2
            or later['stamp_ns'] <= robot['stamp_ns']):
        raise RuntimeError('P2: base is not stationary in the validated station corridor')

    print('PHASE open', flush=True)
    report['open'] = grasp_io.gripper(node, grip, args.open_grip, args.effort)
    if report['open']['status'] != 4 or grasp_io.jaw(grasp_io.read())[1] < .100:
        raise RuntimeError('Gripper did not physically open')
    pregrasp = box_base - approach * (args.pregrasp + TCP_NOMINAL)   # 爪心退到箱心外 pregrasp
    print('PHASE pregrasp', flush=True)
    report['pregrasp'] = grasp_io.move_ee(node, move, pregrasp, quaternion, radius=.005)
    if report['pregrasp']['error_code'] != 1:
        print(json.dumps(dict(report, aborted='预抓取规划失败'), ensure_ascii=False, indent=1))
        return 1
    # Keep target geometry; permit only the fingers to contact it.
    report['scene'] = grasp_io.allow_finger_contact(node, 'pick_bin')

    # 自标定「末端 -> 爪心」偏移：姿态一变它就变，所以每次按当前姿态量，不写死 0.114
    scene = grasp_io.read()
    center, _ = grasp_io.jaw(scene)
    offset = center - grasp_io.base_point(scene, grasp_io.EE)
    insert = tuple(box_base + approach * args.tip_past - offset)
    print('PHASE insert', flush=True)
    report['insert'] = grasp_io.cartesian(node, insert, quaternion)
    if report['insert']['error_code'] != 1:
        print(json.dumps(dict(report, aborted='插入规划失败'), ensure_ascii=False, indent=1))
        return 1

    # 对准：目标「爪心 = 箱心 + tip_past」。箱子被碰一下就会挪，所以**每步重量**、差多少补多少；
    # 但一次最多挪 `--nudge`（默认 8 mm）：E3 实测一次挪 25 mm，那个目标反而规划不出来。
    # 同时只补**垂直于接近方向**的分量——沿接近方向的误差直接补就是继续往下压。
    perpendicular = np.eye(3) - np.outer(approach, approach)
    for _ in range(6):
        scene = grasp_io.read()
        center, gap = grasp_io.jaw(scene)
        box_now = grasp_io.model_base(scene, 'pick_bin')
        error = box_now + approach * args.tip_past - center
        fix = perpendicular @ error
        depth = float(error @ approach)
        report['align'] = {'lateral_error_mm': round(1000 * float(np.linalg.norm(fix)), 1),
                           'depth_error_mm': round(1000 * depth, 1),
                           'gap_mm': round(1000 * gap, 1),
                           'jaw_center_base': [round(float(v), 4) for v in center],
                           'box_center_base': [round(float(v), 4) for v in box_now],
                           'box_drift_mm': round(1000 * float(np.linalg.norm(
                               box_now - box_base)), 1)}
        if np.linalg.norm(fix) <= 0.008 and abs(depth) <= 0.010:
            break
        delta = fix + approach * max(-args.nudge, min(args.nudge, depth))
        if np.linalg.norm(delta) > args.nudge:
            delta = delta / np.linalg.norm(delta) * args.nudge
        insert = tuple(grasp_io.base_point(scene, grasp_io.EE) + delta)
        report['nudge'] = grasp_io.cartesian(node, insert, quaternion)
        if report['nudge']['error_code'] != 1:
            break

    if report['align']['lateral_error_mm'] > 8 or abs(report['align']['depth_error_mm']) > 10:
        raise RuntimeError('P4 alignment failed; closing is forbidden')
    if report['align']['box_drift_mm'] > 10:
        raise RuntimeError('P4 insertion moved the object; closing is forbidden')
    before = grasp_io.read()['pick_bin']['position']
    print('PHASE close', flush=True)
    if args.close_servo:
        history = report['close_servo'] = []
        closing_deadline = time.monotonic()+90.
        for command in np.arange(.10, args.close_grip+.099, .10):
            if time.monotonic() > closing_deadline:
                raise TimeoutError('Closing servo exceeded wall budget')
            command = min(float(command), args.close_grip)
            print('PHASE close target %.2f' % command, flush=True)
            closed = grasp_io.gripper(node, grip, command, args.effort)
            record = dict(target=command, result=closed, corrections=[])
            history.append(record)
            if closed['status'] != 4:
                raise RuntimeError('Incremental closure failed')
            for correction in range(4):
                if time.monotonic() > closing_deadline:
                    raise TimeoutError('Closing compensation exceeded wall budget')
                physical = grasp_io.read()
                box_now = grasp_io.model_base(physical, 'pick_bin')
                if np.linalg.norm(np.subtract(physical['pick_bin']['position'], before)) > .010:
                    raise RuntimeError('Closing servo moved object beyond 10 mm')
                center_now, _ = grasp_io.jaw(physical)
                delta = box_now + approach*args.tip_past - center_now
                record['tcp_error_mm'] = float(np.linalg.norm(delta))*1000
                if np.linalg.norm(delta) <= .004:
                    break
                if correction == 3:
                    raise RuntimeError('Closing TCP compensation did not converge')
                delta *= min(1., .006/np.linalg.norm(delta))
                target = grasp_io.base_point(physical, grasp_io.EE)+delta
                record['corrections'].append(grasp_io.cartesian(node, target, quaternion))
            report['close'] = closed
            if closed['stalled'] or command >= args.close_grip:
                break
    else:
        if args.close_mode == 'contact':
            # 两次命令，不再做增量探测。
            # 为什么不做增量：10-24 实测，连续发 0.008/0.016/0.024 三个小目标时，facade 的
            # JTC 斜坡反而把关节越走越开（报 -0.0004、-0.0119 并 stall），增量探测和它打架。
            # 已验证可靠的是「一次性合到 stall」：stall 的位置就是物理接触点（80 mm 箱子 ≈0.0758）。
            # 问题是之后目标仍在 0.15，多压 0.074 rad 会持续把箱子往下楔（试验后被挤掉）。
            # 所以：先合到 stall 拿到接触点，再用「接触点 + 小预紧」作为保持目标，把多余压力去掉。
            first = grasp_io.gripper(node, grip, args.close_grip, args.effort)
            report['close_first'] = first
            contact = first.get('position')
            if first['status'] != 4 or contact is None:
                raise RuntimeError('Closure search failed: '+str(first))
            if not first.get('stalled'):
                raise RuntimeError('Gripper closed without contact; no object between the pads')
            report['contact_rad'] = round(float(contact), 4)
            report['close'] = grasp_io.gripper(node, grip, contact + args.preload, args.effort)
        else:
            report['close'] = grasp_io.gripper(node, grip, args.close_grip, args.effort)
    scene = grasp_io.read()
    center, gap = grasp_io.jaw(scene)
    report['P5'] = {'gap_mm': round(1000 * gap, 1),
                    'box_pushed_mm': round(1000 * float(np.linalg.norm(
                        np.subtract(scene['pick_bin']['position'], before))), 1),
                    'jaw_center_base': [round(float(v), 4) for v in center],
                    'box_center_base': [round(float(v), 4)
                                        for v in grasp_io.model_base(scene, 'pick_bin')]}
    # Both modes require a completed controller action. Success is not evidence
    # of grip force: P6 checks physical following, and suction_approx explicitly
    # supplies kinematic support instead of contact forces.
    # Even the kinematic approximation must not proceed with an aborted or
    # unconfirmed controller action. It uses a non-contact jaw animation instead.
    close_ok = report['close']['status'] == 4
    if not close_ok or gap < .045 or report['P5']['box_pushed_mm'] > 10:
        raise RuntimeError('P5 contact failed; lifting is forbidden: '+str(report['P5']))

    # Explicit --suction enables kinematic support; no physical fixed joint is made.
    if args.suction:
        report['suction'] = grasp_io.suction(node, True)
        print('PHASE suction attach', flush=True)

    heights = []
    report['P6'] = dict(steps=heights)
    for _ in range(args.steps):
        print('PHASE lift %d' % (len(heights)+1), flush=True)
        lift_scene = grasp_io.read()
        previous_z = lift_scene['pick_bin']['position'][2]
        current_ee = grasp_io.base_point(lift_scene, grasp_io.EE)
        target_ee = current_ee + np.array([0., 0., args.step])
        ee_z = float(target_ee[2])
        lift = grasp_io.cartesian(node, target_ee, quaternion)
        step_scene = grasp_io.read()
        heights.append({'target_z': round(ee_z, 4),
                        'box_z': round(step_scene['pick_bin']['position'][2], 4),
                        'error_code': lift['error_code']})
        if lift['error_code'] != 1 or abs(step_scene['pick_bin']['position'][2]-previous_z-args.step) > .010:
            raise RuntimeError('P6 object did not follow lift: '+str(heights[-1]))
    report['P6'] = {'steps': heights,
                    'rise_mm': round(1000 * (heights[-1]['box_z'] - before[2]), 1)}
    # P6b：抬升结束后静置，看箱子会不会像 10-24 那次一样被持续挤压而滑落。
    if args.hold > 0:
        print('PHASE hold %.0fs' % args.hold, flush=True)
        held_z = grasp_io.read()['pick_bin']['position'][2]
        time.sleep(args.hold)
        later_z = grasp_io.read()['pick_bin']['position'][2]
        report['P6b'] = {'hold_s': args.hold, 'z_at_lift': round(held_z, 4),
                         'z_after_hold': round(later_z, 4),
                         'drop_mm': round(1000 * (held_z - later_z), 1)}
        if held_z - later_z > .005:
            raise RuntimeError('P6b object did not stay held: '+str(report['P6b']))
    report['passed'] = (args.steps >= 3 and report['P6']['rise_mm'] >= 50
                        and report.get('P6b', {}).get('drop_mm', 0) <= 5)
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
