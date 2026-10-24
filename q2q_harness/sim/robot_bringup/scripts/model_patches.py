"""对 Clearpath 生成树施加的三个补丁 + 校验。

为什么需要这个文件：base_footprint 在 A200 的 URDF 里是 base_link 的**子**节点
（方向反了），而 Nav2 / MoveIt 的惯例是 base_footprint 在最上面当根。
父链一改，另外两个配置（ekf、差速控制器）必须跟着改，否则 base_link 会出现两个父。
详见同目录的 补丁说明.md。

调用方：prepare_setup.py（在生成器跑完之后调用）。
"""

import os
import subprocess
import xml.etree.ElementTree as ET

URDF_XACRO = 'robot.urdf.xacro'        # RSP 实际读的文件名（description.launch.py 写死的）
URDF_RAW = 'robot.raw.urdf'            # 展开后的原始模型（中间产物）
URDF_PATCHED = 'robot.patched.urdf'    # 打过补丁的最终模型
EKF_YAML = 'platform/config/localization.yaml'
CTRL_YAML = 'platform/config/control.yaml'
SENSOR_YAML = 'sensors/config/lidar2d_0.yaml'

ROOT_JOINT = 'base_footprint_joint'
NEW_ROOT = 'base_footprint'
NEW_CHILD = 'base_link'
EKF_KEY = 'base_link_frame'
CTRL_KEY = 'base_frame_id'
LIDAR_SENSOR = 'lidar2d_0'


def _path(setup, rel):
    return os.path.join(setup, rel)


def _expand_urdf(setup, namespace):
    """用 xacro 把生成物展开成纯 URDF —— 补丁只能在展开之后打，
    因为那个关节藏在 a200 宏里，改宏文件不现实。

    ⚠️ 这 6 个参数必须和 description.launch.py 里那条 xacro 命令一模一样：
    少了 is_sim:=true，展开出来的是「真实机器人串口硬件」版本，
    Gazebo 里就没有 ign_ros2_control 插件，controller_manager 不存在，
    所有控制器 spawn 都会失败（这个坑踩过一次，见 补丁说明.md 第 6 节）。
    """
    raw = _path(setup, URDF_RAW)
    command = [
        'xacro', _path(setup, URDF_XACRO),
        'is_sim:=true',
        'gazebo_controllers:=%s' % _path(setup, 'platform/config/control.yaml'),
        'namespace:=%s' % namespace,
        'use_fake_hardware:=false',
        # 必须是 true：false 时机械臂关节不会被写进 ros2_control，没有命令接口，
        # arm_0_joint_trajectory_controller / gripper 会在激活时报
        # "Not acceptable command interfaces combination"（踩过一次）
        'use_manipulation_controllers:=true',
        'use_platform_controllers:=true',
    ]
    with open(raw, 'w', encoding='utf-8') as out:
        result = subprocess.run(command, stdout=out, stderr=subprocess.PIPE, text=True)
    if result.returncode != 0:
        raise SystemExit('展开 URDF 失败：%s' % result.stderr.strip()[:400])
    return raw


def patch_lidar_rate(root, rate):
    """激光的 update_rate 在生成器里写死成 50 Hz（Lidar2dDescription），
    YAML 传不进去，只能在这里改。软渲染下 50 Hz 是 CPU 大头。

    顺手把传感器类型从 `gpu_lidar` 改成 `lidar`（CPU 射线求交）：
    2026-10-21 实测，这台 VM 走 llvmpipe 软渲染，`gpu_lidar` 的 720 束里只有 8 束有值、
    而且值全是 24.999（= range_max，等于「什么都没打到」）——地图因此一个障碍格都出不来
    （自由 15.6% / 未知 84.4% / 障碍 0%）。CPU 激光直接扫物理碰撞体，不依赖图形后端。
    """
    touched = 0
    for gazebo in root.findall('gazebo'):
        for sensor in gazebo.findall('sensor'):
            if sensor.get('name') != LIDAR_SENSOR:
                continue
            node = sensor.find('update_rate')
            if node is None:
                node = ET.SubElement(sensor, 'update_rate')
            node.text = str(rate)
            sensor.set('type', 'gpu_lidar')     # 保持 GPU 激光（CPU 激光在 Fortress 里不出版数据，实测无消息）
            # 但**关掉它**：渲染激光在这台 VM 上打不到任何东西（720 束里 8 束、值全 24.999），
            # 而 Sensors 插件仍会为它每帧渲染，把仿真拖到 3% 实时（实测）。
            # 激光数据由 scripts/virtual_lidar.py 用几何射线提供。
            always_on = sensor.find('always_on')
            if always_on is None:
                always_on = ET.SubElement(sensor, 'always_on')
            always_on.text = 'false'
            touched += 1
    if touched != 1:
        raise SystemExit('补丁失败：找到 %d 个 %s 传感器（期望 1 个）' % (touched, LIDAR_SENSOR))


def patch_gripper_mimic(root):
    """Preserve q=multiplier*master+offset using a zero-offset joint coordinate.

    Fortress's 0.7.21 mimic backend ignores offset. For these z-axis joints,
    bake Rz(axis_z*offset) into origin and use q_new=q_old-offset, including
    transformed limits. FK and collision geometry remain equivalent; no fixed
    finger or attachment approximation is introduced. Reapplying is a no-op.
    """
    changed = 0
    for block in root.findall('ros2_control'):
        if 'gripper' not in (block.get('name') or ''):
            continue
        for joint in list(block.findall('joint')):
            name = joint.get('name') or ''
            physical = root.find("joint[@name='%s']" % name)
            mimic = physical.find('mimic') if physical is not None else None
            if mimic is None:
                continue
            offset = float(mimic.get('offset', '0'))
            if offset:
                axis = physical.find('axis').get('xyz').split()
                origin = physical.find('origin')
                rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
                if axis[:2] != ['0', '0'] or any(abs(v) > 1e-9 for v in rpy[:2]):
                    raise ValueError('Offset baking only supports fixed-axis z tip joints')
                rpy[2] += float(axis[2]) * offset
                origin.set('rpy', ' '.join(str(v) for v in rpy))
                limits = physical.find('limit')
                for key in ('lower', 'upper'):
                    limits.set(key, str(float(limits.get(key)) - offset))
                mimic.set('offset', '0')
            values = {'mimic': mimic.get('joint'),
                      'multiplier': mimic.get('multiplier', '1')}
            for param in list(joint.findall('param')):
                joint.remove(param)
            for key, value in values.items():
                node = ET.SubElement(joint, 'param', {'name': key})
                node.text = value
            changed += 1
    return changed


def patch_gripper_contact(root):
    """指面摩擦 + 手指关节阻尼（2026-10-24 夹持方案 A）。

    为什么动这两个数：
    1. **摩擦**：指面和箱子的碰撞摩擦原先都取默认值，引擎里没有区分。外部资料（见
       docs/evaluations/2026-10-24-夹持问题外部资料调研.md）的结论是「夹爪 mu 要大于物体 mu」，
       所以指面显式取 2.0，箱子在世界里显式取 1.0。
    2. **阻尼**：四个手指关节原本一条 `<dynamics>` 都没有。位置控制被物体挡住时，
       无阻尼会让 stall 判定抖得很厉害（实测同一命令三次停在 0.048/0.068/0.076）。
    这两项都是参数正确性，不改变几何，也不替代摩擦抓取。
    """
    fingers = ('arm_0_gripper_left_finger_prox_link', 'arm_0_gripper_left_finger_dist_link',
               'arm_0_gripper_right_finger_prox_link', 'arm_0_gripper_right_finger_dist_link')
    joints = ('arm_0_gripper_left_finger_bottom_joint', 'arm_0_gripper_right_finger_bottom_joint',
              'arm_0_gripper_left_finger_tip_joint', 'arm_0_gripper_right_finger_tip_joint')
    changed = 0
    for link in fingers:
        if root.find('link[@name="%s"]' % link) is None:
            continue
        gazebo = root.find("gazebo[@reference='%s']" % link)
        if gazebo is None:
            gazebo = ET.SubElement(root, 'gazebo', {'reference': link})
        for key, value in {'mu1': '2.0', 'mu2': '2.0'}.items():
            node = gazebo.find(key)
            if node is None:
                node = ET.SubElement(gazebo, key)
            node.text = value
        changed += 1
    for joint in root.findall('joint'):
        if joint.get('name') not in joints:
            continue
        dynamics = joint.find('dynamics')
        if dynamics is None:
            dynamics = ET.SubElement(joint, 'dynamics')
        dynamics.set('damping', '0.1')
        dynamics.set('friction', '0.0')
        changed += 1
    return changed


def patch_suction_gripper(root):
    """给机器人模型挂 Gazebo 的 `DetachableJoint` 插件 —— **吸附式夹持（真空吸近似）**。

    为什么需要它：2F Lite 的摩擦夹持在本模型精度下不可复现（见
    docs/evaluations/2026-10-24-成熟夹持方案评估.md）。`DetachableJoint` 是 Gazebo 自带的
    标准机制，对应真实工业里的真空吸盘：接触后把物体与末端刚性连接，松开时断开。

    **必须标注**：这不是摩擦抓取。抓取 JSON 会带 `grasp_mode=suction_approx`。
    只由环境变量 `Q2Q_GRIPPER_MODE=suction` 启用，默认关闭——默认仍是原来的摩擦路径。
    """
    for gazebo in root.findall('gazebo'):
        for plugin in gazebo.findall('plugin'):
            if 'DetachableJoint' in (plugin.get('name') or ''):
                return 0                       # 幂等：已经打过
    gazebo = ET.SubElement(root, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin', {
        'filename': 'libignition-gazebo6-detachable-joint-system.so',
        'name': 'ignition::gazebo::systems::DetachableJoint'})
    for key, value in {
        'parent_link': 'arm_0_gripper_left_finger_dist_link',
        'child_model': 'pick_bin',
        'child_link': 'link',
        'attach_topic': '/q2q/suction/attach',
        'detach_topic': '/q2q/suction/detach',
        'suppress_child_warning': 'true',
    }.items():
        ET.SubElement(plugin, key).text = value
    return 1


def patch_urdf(setup, namespace, lidar_rate):
    """把 base_link -> base_footprint 翻转成 base_footprint -> base_link，偏移取反。"""
    root = ET.parse(_expand_urdf(setup, namespace)).getroot()

    flipped = 0
    for joint in root.findall('joint'):
        if joint.get('name') != ROOT_JOINT:
            continue
        parent = joint.find('parent').get('link')
        if parent == NEW_ROOT:            # 已经是目标状态，幂等跳过
            flipped = 1
            break
        origin = joint.find('origin')
        xyz = [float(v) for v in origin.get('xyz').split()]
        if abs(xyz[0]) > 1e-9 or abs(xyz[1]) > 1e-9:
            raise SystemExit('补丁中止：%s 不只有 z 偏移，逻辑不成立 %s' % (ROOT_JOINT, xyz))
        origin.set('xyz', ' '.join('%.6f' % -v for v in xyz))
        joint.find('parent').set('link', NEW_ROOT)
        joint.find('child').set('link', NEW_CHILD)
        flipped = 1
        break

    if flipped != 1:
        # Jackal(j100) 这类平台**根本没有** base_footprint：raw URDF 的根就是 base_link。
        # A200 是「已有 base_footprint 链、方向反了要翻」，Jackal 是「没有、得新建」，
        # 两种逻辑必须分开，否则换平台第一步就中止（2026-10-23 实测）。
        links = {l.get('name') for l in root.findall('link')}
        if NEW_ROOT in links:
            raise SystemExit('补丁失败：已有 %s 却没有 %s，逻辑不成立' % (NEW_ROOT, ROOT_JOINT))
        ET.SubElement(root, 'link', {'name': NEW_ROOT})
        joint = ET.SubElement(root, 'joint', {'name': ROOT_JOINT, 'type': 'fixed'})
        ET.SubElement(joint, 'parent', {'link': NEW_ROOT})
        ET.SubElement(joint, 'child', {'link': NEW_CHILD})
        # Jackal's wheel centres are base_z=0.0345 m, radius=0.098 m.
        # Ground projection must be 0.0635 m below base_link, not coincident.
        wheel = root.find("joint[@name='front_left_wheel_joint']/origin")
        if wheel is None:
            raise ValueError('New platform needs an explicit ground-projection height')
        wheel_xyz = [float(v) for v in wheel.get('xyz').split()]
        if any(abs(a-b) > 1e-6 for a,b in zip(wheel_xyz, [.131, .187795, .0345])):
            raise ValueError('Unknown wheel geometry; revalidate ground projection')
        height = '0.0635'
        ET.SubElement(joint, 'origin', {'xyz': '0 0 '+height, 'rpy': '0 0 0'})
        flipped = 2

    patch_lidar_rate(root, lidar_rate)
    gripper = patch_gripper_mimic(root)
    if os.environ.get('Q2Q_GRIPPER_MODE', 'mimic') == 'commanded':
        # Keep URDF mimic for unloaded kinematics, but command each physical joint
        # from the same desired closure. Upstream's measured-master follower loses
        # opposing preload when contact blocks the master. This is an explicitly
        # actuated simulation surrogate, not a claim about four real motors.
        for control in root.findall('ros2_control'):
            if 'gripper' not in control.get('name', ''):
                continue
            for joint in control.findall('joint'):
                for parameter in list(joint.findall('param')):
                    joint.remove(parameter)
                if joint.find("command_interface[@name='position']") is None:
                    ET.SubElement(joint, 'command_interface', {'name': 'position'})
                for interface in ['position', 'velocity']:
                    if joint.find("state_interface[@name='%s']" % interface) is None:
                        ET.SubElement(joint, 'state_interface', {'name': interface})
    gain = float(os.environ.get('Q2Q_POSITION_GAIN', '.1'))
    if not .05 <= gain <= 1.:
        raise ValueError('Q2Q_POSITION_GAIN must be within [.05,1]')
    if abs(gain-.1) > 1e-9:
        for control in root.findall('ros2_control'):
            if not any(j.get('name', '').startswith('arm_0_') for j in control.findall('joint')):
                continue
            hardware = control.find('hardware')
            hardware.find('plugin').text = 'robot_sim_control/GainSystem'
            parameter = ET.SubElement(hardware, 'param', {'name': 'q2q_position_gain'})
            parameter.text = str(gain)
    if os.environ.get('Q2Q_GRIPPER_COLLISION', 'mesh') == 'convex':
        import collision_hulls
        collision_hulls.apply(root, setup)
    print('[patches] 夹爪 mimic 已对齐零偏置物理坐标：%d 个关节' % gripper)
    contact = patch_gripper_contact(root)
    print('[patches] 指面摩擦 mu=2.0 / 手指关节 damping=0.1：%d 处' % contact)
    if os.environ.get('Q2Q_GRIPPER_MODE', 'mimic') == 'suction':
        print('[patches] 吸附式夹持插件：%d 处（grasp_mode=suction_approx）'
              % patch_suction_gripper(root))

    # 轮编码器积分无法反映打滑。用 Gazebo 原生插件发布独立的物理真值里程计，
    # 保留 platform/odom 作为轮式估计；导航反馈必须跟随真实车体。
    for plugin in root.findall('.//plugin'):
        if plugin.get('name') == 'ignition::gazebo::systems::WheelSlip':
            for wheel in plugin.findall('wheel'):
                wheel.find('slip_compliance_longitudinal').text = '0.01'
                wheel.find('slip_compliance_lateral').text = '0.5'
                # WheelSlip 要求第一摩擦方向平行轮轴。轮碰撞圆柱经 Rx(pi/2)
                # 转到 Y 轴，因此碰撞局部 Z 平行轮轴；原模型没有设置 fdir1。
                surface = root.find("gazebo[@reference='%s']" % wheel.get('link_name'))
                if surface is None:
                    surface = ET.SubElement(root, 'gazebo', {'reference': wheel.get('link_name')})
                for key, value in {'fdir1': '0 0 1', 'mu1': '0.5', 'mu2': '1.0'}.items():
                    node = surface.find(key)
                    if node is None:
                        node = ET.SubElement(surface, key)
                    node.text = value
    for gazebo in root.findall('gazebo'):
        removed = False
        for old in list(gazebo.findall('plugin')):
            if old.get('name') == 'ignition::gazebo::systems::OdometryPublisher':
                gazebo.remove(old)
                removed = True
        if removed and len(gazebo) == 0:
            root.remove(gazebo)
    gazebo = ET.SubElement(root, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin', {
        'filename': 'libignition-gazebo-odometry-publisher-system.so',
        'name': 'ignition::gazebo::systems::OdometryPublisher'})
    for key, value in {
        'odom_frame': 'odom', 'robot_base_frame': 'base_footprint',
        'odom_publish_frequency': '25', 'dimensions': '2',
        'odom_topic': '/model/%s/robot/ground_truth_odom' % namespace,
        'tf_topic': '/model/%s/robot/ground_truth_tf' % namespace,
    }.items():
        ET.SubElement(plugin, key).text = value

    patched = _path(setup, URDF_PATCHED)
    ET.ElementTree(root).write(patched, encoding='utf-8', xml_declaration=True)

    # 用「只做 include 的包装器」顶掉原 xacro：RSP 读的还是这个文件名，内容换成打过补丁的模型
    with open(_path(setup, URDF_XACRO), 'w', encoding='utf-8') as fh:
        fh.write('<?xml version="1.0"?>\n'
                 '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="%s">\n'
                 '  <!-- 由 robot_bringup/scripts/model_patches.py 生成，不要手改 -->\n'
                 '  <xacro:include filename="%s"/>\n'
                 '</robot>\n' % (root.get('name'), patched))


def _replace_quoted(path, key, new_value):
    """把 `key: 'old'` 这种整行里的引号值换掉；命中数不是 1 就中止。"""
    with open(path, 'r', encoding='utf-8') as fh:
        lines = fh.readlines()

    hits = 0
    for i, line in enumerate(lines):
        if line.strip().startswith(key + ':'):
            head, rest = line.split(':', 1)
            old = rest.strip()
            lines[i] = '%s: \'%s\'\n' % (head, new_value)
            if old == "'%s'" % new_value:
                hits = 1            # 已经是目标值，幂等
                break
            hits += 1
        if hits > 1:
            raise SystemExit('补丁中止：%s 里 %s 出现多次' % (path, key))

    if hits == 0:
        raise SystemExit('补丁失败：%s 里找不到 %s:' % (path, key))

    with open(path, 'w', encoding='utf-8') as fh:
        fh.writelines(lines)


def _replace_flag(path, key, flag):
    """把 `key: True/False` 这种不带引号的布尔行换掉；命中数不是 1 就中止。"""
    with open(path, 'r', encoding='utf-8') as fh:
        lines = fh.readlines()

    want = 'True' if flag else 'False'
    hits = 0
    for i, line in enumerate(lines):
        if line.strip().startswith(key + ':'):
            head = line.split(':', 1)[0]
            lines[i] = '%s: %s\n' % (head, want)
            hits += 1
        if hits > 1:
            raise SystemExit('补丁中止：%s 里 %s 出现多次' % (path, key))
    if hits == 0:
        raise SystemExit('补丁失败：%s 里找不到 %s:' % (path, key))

    with open(path, 'w', encoding='utf-8') as fh:
        fh.writelines(lines)


def _replace_scalar(path, key, text):
    """把 `key: 任意值` 整行换成 `key: text`（不加引号，用于数值参数）。"""
    with open(path, 'r', encoding='utf-8') as fh:
        lines = fh.readlines()

    hits = 0
    for i, line in enumerate(lines):
        if line.strip().startswith(key + ':'):
            head = line.split(':', 1)[0]
            lines[i] = '%s: %s\n' % (head, text)
            hits += 1
        if hits > 1:
            raise SystemExit('补丁中止：%s 里 %s 出现多次' % (path, key))
    if hits == 0:
        raise SystemExit('补丁失败：%s 里找不到 %s:' % (path, key))

    with open(path, 'w', encoding='utf-8') as fh:
        fh.writelines(lines)


def patch_ekf(setup):
    """EKF 只把 platform/odom 抄成 odom→base_footprint（本项目没装 IMU，它没有融合对象）。
    50 Hz 抄一遍纯属浪费：这台 VM 上它经常报 Failed to meet update rate，
    TF 一落后，slam_toolbox 的 MessageFilter（队列长度 1）就把整帧激光丢掉。
    当前基线：EKF 和差速控制器都不发 odom TF，由 Gazebo 物理真值插件统一发布。
    少一个节点、少一次 DDS 往返，TF 与里程计同源同时刻。"""
    _replace_quoted(_path(setup, EKF_YAML), EKF_KEY, NEW_ROOT)
    _replace_flag(_path(setup, EKF_YAML), 'publish_tf', False)
    _replace_scalar(_path(setup, EKF_YAML), 'frequency', '20.0')


def patch_diff_drive(setup):
    _replace_quoted(_path(setup, CTRL_YAML), CTRL_KEY, NEW_ROOT)
    # 保留轮编码器 odom 供对比；TF 使用独立的 Gazebo 物理真值，避免打滑造成假到位。
    _replace_flag(_path(setup, CTRL_YAML), 'enable_odom_tf', False)
    # 原 1.875 是另一套打滑模型的经验补偿；本基线的原地转向实测需约 1.2。
    _replace_scalar(_path(setup, CTRL_YAML), 'wheel_separation_multiplier', '1.2')


def patch_gripper_controller(setup):
    """Ramp gripper position with JTC; preserve the GripperCommand API via facade."""
    import yaml
    path = _path(setup, CTRL_YAML)
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    namespace = next(iter(data.values()))
    manager = namespace['controller_manager']['ros__parameters']
    manager['arm_0_gripper_controller.type'] = 'joint_trajectory_controller/JointTrajectoryController'
    joints = ['arm_0_gripper_right_finger_bottom_joint']
    if os.environ.get('Q2Q_GRIPPER_MODE', 'mimic') == 'commanded':
        joints += sorted(['arm_0_gripper_left_finger_bottom_joint',
                          'arm_0_gripper_left_finger_tip_joint', 'arm_0_gripper_right_finger_tip_joint'])
    parameters = {
        'use_sim_time': True, 'joints': joints,
        'command_interfaces': ['position'], 'state_interfaces': ['position', 'velocity'],
        'constraints.stopped_velocity_tolerance': 0.0, 'constraints.goal_time': 2.0,
        # Contact can prevent reaching the free-closing angle. Keep final position
        # command; the facade reports reached/stalled using measured position.
        'constraints.arm_0_gripper_right_finger_bottom_joint.goal': 1.0}
    for joint in joints:
        parameters['constraints.'+joint+'.goal'] = 1.0
    namespace['arm_0_gripper_controller']['ros__parameters'] = parameters
    with open(path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream, sort_keys=False)


def patch_sensor_topic(setup):
    """把渲染激光的桥接话题改名成 scan_raw，把 scan 让给 virtual_lidar.py。

    2026-10-21：这台 VM 的 GPU 渲染拿不到深度（720 束里 8 束有值、且全 = 24.999），
    激光改用几何射线（scripts/virtual_lidar.py）。原桥接还在，所以挪到 scan_raw，
    避免同一个话题上有两个发布者。
    """
    path = _path(setup, SENSOR_YAML)
    with open(path, 'r', encoding='utf-8') as fh:
        text = fh.read()
    old = 'ros_topic_name: lidar2d_0/scan'
    new = 'ros_topic_name: lidar2d_0/scan_raw'
    lines = text.splitlines(keepends=True)
    normalized = [line.strip().removeprefix('- ').strip() for line in lines]
    old_count = normalized.count(old)
    new_count = normalized.count(new)
    if old_count == 0 and new_count == 1:
        return
    if old_count != 1 or new_count != 0:
        raise SystemExit('补丁失败：%s 里 %s 出现 %d 次（期望 1 次）'
                         % (path, old, old_count))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.writelines(line.replace(old, new) if key == old else line
                      for line, key in zip(lines, normalized))


def verify(setup, lidar_rate):
    """补丁是否真的生效。任何一条不成立就中止启动。"""
    patched = _path(setup, URDF_PATCHED)
    root = ET.parse(patched).getroot()

    with open(patched, 'r', encoding='utf-8') as fh:
        text = fh.read()
    # 这一条是防「展开时漏参数」：漏了 is_sim，模型里就没有仿真硬件接口
    if 'ign_ros2_control/IgnitionSystem' not in text:
        raise SystemExit('校验失败：模型里没有 ign_ros2_control 仿真接口（展开时参数漏了？）')
    if 'IgnitionROS2ControlPlugin' not in text:
        raise SystemExit('校验失败：模型里没有 Gazebo 控制器插件（展开时 gazebo_controllers 漏了？）')
    if 'arm_0_joint_1' in text and 'command_interface name="position"' not in text:
        raise SystemExit('校验失败：机械臂关节没有 position 命令接口'
                         '（use_manipulation_controllers 应为 true）')

    children = [j.find('child').get('link') for j in root.findall('joint')]
    parents = {}
    for joint in root.findall('joint'):
        parents.setdefault(joint.find('child').get('link'), []).append(joint.find('parent').get('link'))

    links = [l.get('name') for l in root.findall('link')]
    roots = [name for name in links if name not in children]
    multi = {k: v for k, v in parents.items() if len(v) > 1}

    if roots != [NEW_ROOT]:
        raise SystemExit('校验失败：根应该是 %s，实际是 %s' % (NEW_ROOT, roots))
    if len(parents.get(NEW_CHILD, [])) != 1 or parents[NEW_CHILD][0] != NEW_ROOT:
        raise SystemExit('校验失败：%s 的父应该是 %s，实际是 %s'
                         % (NEW_CHILD, NEW_ROOT, parents.get(NEW_CHILD)))
    if multi:
        raise SystemExit('校验失败：这些 link 有多个父 %s' % multi)
    # 阈值按「确实带上机械臂」取：A200 是 34 个 link、j100(Jackal) 是 28 个，
    # 20 以下通常是模型缺件（臂/底盘没生成），不是平台差异。
    if len(links) < 20:
        raise SystemExit('校验失败：link 只有 %d 个，模型像是缺件' % len(links))

    def values(path, key):
        import yaml
        with open(path, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        def walk(item):
            found = []
            if isinstance(item, dict):
                for name, value in item.items():
                    found.extend([value] if name == key else walk(value))
            return found
        return walk(data)

    for path, key in ((_path(setup, EKF_YAML), EKF_KEY), (_path(setup, CTRL_YAML), CTRL_KEY)):
        if values(path, key) != [NEW_ROOT]:
            raise SystemExit('校验失败：%s 里 %s 不是 %s' % (path, key, NEW_ROOT))

    rates = [s.findtext('update_rate') for g in root.findall('gazebo')
             for s in g.findall('sensor') if s.get('name') == LIDAR_SENSOR]
    if rates != [str(lidar_rate)]:
        raise SystemExit('校验失败：激光 update_rate 应为 %s，实际是 %s' % (lidar_rate, rates))

    # 保留关闭状态的 gpu_lidar 描述；实际 scan 由二维几何射线节点提供。
    kinds = [s.get('type') for g in root.findall('gazebo')
             for s in g.findall('sensor') if s.get('name') == LIDAR_SENSOR]
    if kinds != ['gpu_lidar']:
        raise SystemExit('校验失败：激光类型应为 gpu_lidar，实际是 %s' % kinds)

    # 渲染激光必须关着：它拿不到数据，却仍然消耗渲染线程（实测能把 RTF 从 0.4 打到 0.03）
    flags_on = [s.findtext('always_on') for g in root.findall('gazebo')
                for s in g.findall('sensor') if s.get('name') == LIDAR_SENSOR]
    if flags_on != ['false']:
        raise SystemExit('校验失败：激光 always_on 应为 false（由虚拟激光替代），实际是 %s' % flags_on)

    # odom TF 只由 Gazebo 真值插件发布；差速控制器和 ekf 都不得重复发布。
    # 两个都发会出现同一段 odom->base_footprint 两个源，TF 里"谁后到听谁的"，
    # 叠加起来就是抖动 + 时间戳跳变（slam 丢帧的另一种成因）。
    flags = ((_path(setup, CTRL_YAML), 'enable_odom_tf', False),
             (_path(setup, EKF_YAML), 'publish_tf', False))
    for path, key, want in flags:
        value = values(path, key)
        if value != [want]:
            raise SystemExit('校验失败：%s 里 %s 应为 %s，实际是 %s'
                             % (path, key, want, value))

    if root.find(".//plugin[@name='ignition::gazebo::systems::OdometryPublisher']") is None:
        raise SystemExit('校验失败：缺少仿真物理真值里程计插件')
    print('[patches] 校验通过：根=%s  link=%d  joint=%d  多父=无  激光=%s Hz  TF 发布者=Gazebo真值'
          % (roots[0], len(links), len(root.findall('joint')), lidar_rate))


def apply_all(setup, namespace, lidar_rate):
    patch_urdf(setup, namespace, lidar_rate)
    patch_ekf(setup)
    patch_diff_drive(setup)
    patch_gripper_controller(setup)
    patch_sensor_topic(setup)
