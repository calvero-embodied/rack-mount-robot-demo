"""Geometry and transport helpers for the lecture's Gazebo Ignition 6."""
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from setup_scene import box, rack, server

HERE = Path(__file__).resolve().parent
BASE_Z = 1.015
LOAD = (0.17, 0.0, 0.21)
HOLD = (0.23, 0.0, 0.21)
SIZES = {'1u': {'height': 0.018, 'mass': 0.05},
         '2u': {'height': 0.036, 'mass': 0.10}}
LEVELS = {'upper': 0.33, 'lower': 0.03}


def target_for(level):
    return (HOLD[0], HOLD[1], LEVELS[level])


def server_offset(size):
    return 0.002 + SIZES[size]['height']/2


def call(service, reqtype, request, reptype='Boolean'):
    result = subprocess.run(['ign', 'service', '-s', service,
        '--reqtype', f'ignition.msgs.{reqtype}',
        '--reptype', f'ignition.msgs.{reptype}', '--timeout', '5000',
        '--req', request], capture_output=True, text=True, check=True)
    if reptype == 'Boolean' and 'data: true' not in result.stdout:
        raise RuntimeError(f'{service}: {result.stdout} {result.stderr}')
    return result.stdout


def create(root, filename):
    path = HERE / filename
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
    call('/world/default/create', 'EntityFactory', f'sdf_filename: "{path}"')


def remove(name):
    graph = call('/world/default/scene/graph', 'Empty', ' ', 'StringMsg')
    if f'{name} (' in graph:
        call('/world/default/remove', 'Entity', f'name: "{name}", type: MODEL')


def prepare_rack():
    remove('rack_demo_frame')
    root = rack()
    root.find('model/pose').text = '0.26 0 1.015 0 0 0'
    for pose in root.findall('.//link/*/pose'):
        fields = pose.text.split()
        if abs(float(fields[1])) == 0.106:
            fields[1] = str(math.copysign(0.12, float(fields[1])))
        if fields[2] == '0.12':
            fields[2] = '0.24'
        elif fields[2] == '0.006':
            fields[2] = '0.003'
        elif fields[2] == '0.234':
            fields[2] = '0.474'
        pose.text = ' '.join(fields)
    for size in root.findall('.//box/size'):
        if size.text == '0.012 0.012 0.24':
            size.text = '0.012 0.012 0.48'
        elif size.text == '0.012 0.224 0.012':
            size.text = '0.012 0.252 0.012'
    for component in root.findall('.//link/*'):
        pose=component.find('pose'); dims=component.find('geometry/box/size')
        if pose is not None and dims is not None and pose.text.split()[2]=='0.003':
            values=dims.text.split();values[2]='0.006';dims.text=' '.join(values)
    link = root.find('model/link')
    for level, height in LEVELS.items():
        color = '0.1 0.75 1 1' if level == 'upper' else '0.2 1 0.35 1'
        for side in (-0.12, 0.12):
            box(link, f'target_{level}_{side}', f'-0.083 {side} {height+0.002}',
                '0.003 0.016 0.006', color, collision=False)
    create(root, 'demo_frame.sdf')


def place_server(x, y, z, fixed=False, orientation=None, size='1u', name='rack_demo_server'):
    remove(name)
    root = server()
    model = root.find('model')
    model.set('name', name)
    roll = pitch = yaw = 0.0
    if orientation:
        qx, qy, qz, qw = [orientation.get(k,0) for k in ('x','y','z','w')]
        roll = math.atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))
        pitch = math.asin(max(-1, min(1, 2*(qw*qy-qz*qx))))
        yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    model.find('pose').text = f'{x} {y} {z} {roll} {pitch} {yaw}'
    if fixed:
        ET.SubElement(model, 'static').text = 'true'
    height = SIZES[size]['height']
    mass = SIZES[size]['mass']
    for dimensions in model.findall('.//box/size'):
        if dimensions.text == '0.003 0.202 0.018':
            dimensions.text = f'0.003 0.198 {height}'
        elif dimensions.text == '0.12 0.19 0.018':
            dimensions.text = f'0.12 0.19 {height}'
    inertial = model.find('link/inertial')
    inertial.find('mass').text = str(mass)
    for key, value in {'ixx':mass*(0.19**2+height**2)/12,
                       'iyy':mass*(0.12**2+height**2)/12,
                       'izz':mass*(0.12**2+0.19**2)/12}.items():
        inertial.find('inertia/'+key).text = str(value)
    if size == '2u':
        link = model.find('link')
        for visual in link.findall('visual'):
            if visual.get('name','').startswith('port_'):
                xyz = visual.find('pose').text.split()
                xyz[2] = '-0.009'
                visual.find('pose').text = ' '.join(xyz)
        for index in range(8):
            box(link, f'upper_port_{index}',
                f'-0.063 {-0.074 + index*0.021} 0.009',
                '0.001 0.013 0.008', '0.015 0.02 0.025 1', collision=False)
        for material in link.findall('visual/material'):
            if material.find('diffuse').text == '0.05 0.25 0.48 1':
                material.find('diffuse').text = '0.6 0.22 0.05 1'
                material.find('ambient').text = '0.6 0.22 0.05 1'
    create(root, f'{name}.sdf')


def joints(x, y, z, lift=0.0):
    """Analytic IK for a horizontal tray. z is relative to robot base."""
    radial = math.hypot(x, y) - 0.07
    height = z - lift + 0.08 - 0.0714
    a, b = 0.083, 0.0935
    c = (radial**2 + height**2 - a*a - b*b) / (2*a*b)
    if not -1 <= c <= 1:
        raise ValueError('Tray target outside arm workspace')
    bend = -math.acos(c)
    q2 = math.atan2(radial, height) + math.atan2(b*math.sin(bend), a+b*c)
    q3 = bend
    q4 = q2-q3-math.pi/2
    q = [math.atan2(y, x), q2, q3, q4]
    for value, low, high in zip(q, [-2.61, -1.824, -2.556, -1.84],
                               [2.61, 1.783, 2.454, 1.768]):
        if not low <= value <= high:
            raise ValueError(f'Joint limit exceeded: {q}')
    return q


def robot_description():
    from ament_index_python.packages import get_package_share_directory
    import xacro
    share = get_package_share_directory('crane_plus_description')
    doc = xacro.process_file(str(Path(share)/'urdf/crane_plus.urdf.xacro'),
        mappings={'use_gazebo':'true', 'use_ignition':'true',
            'gz_control_config_package':'crane_plus_control',
            'gz_control_config_file_path':'config/crane_plus_controllers.yaml'})
    robot = ET.fromstring(doc.toxml())
    # Lift stage makes high-rack access possible without moving below the table.
    lift_joint = robot.find("joint[@name='base_link_to_world_joint']")
    lift_joint.set('type', 'prismatic')
    lift_joint.set('name', 'rack_lift_joint')
    lift_joint.find('parent').set('link','rack_lift_base')
    anchor=ET.SubElement(robot,'link',name='rack_lift_base')
    inertia=ET.SubElement(anchor,'inertial')
    ET.SubElement(inertia,'mass',value='0.2')
    ET.SubElement(inertia,'inertia',ixx='0.001',iyy='0.001',izz='0.001',
                  ixy='0',ixz='0',iyz='0')
    anchor_joint=ET.SubElement(robot,'joint',name='rack_anchor',type='fixed')
    ET.SubElement(anchor_joint,'parent',link='world')
    ET.SubElement(anchor_joint,'child',link='rack_lift_base')
    ET.SubElement(lift_joint, 'axis', xyz='0 0 1')
    ET.SubElement(lift_joint, 'limit', lower='-0.01', upper='0.31', velocity='0.05', effort='1000')
    control = robot.find('ros2_control')
    cj = ET.SubElement(control, 'joint', name='rack_lift_joint')
    ci = ET.SubElement(cj, 'command_interface', name='position')
    ET.SubElement(ci, 'param', name='min').text = '0'
    ET.SubElement(ci, 'param', name='max').text = '0.30'
    ET.SubElement(cj, 'state_interface', name='position')
    ET.SubElement(cj, 'state_interface', name='velocity')
    for param in robot.findall('gazebo/plugin/parameters'):
        param.text = str(HERE/'rack_controllers.yaml')
    import yaml
    config = yaml.safe_load((Path(get_package_share_directory('crane_plus_control'))/
        'config/crane_plus_controllers.yaml').read_text())
    config['crane_plus_arm_controller']['ros__parameters']['joints'].append('rack_lift_joint')
    (HERE/'rack_controllers.yaml').write_text(yaml.safe_dump(config))
    for parent, name, xyz, size in [
        ('rack_lift_base','lift_mast','-0.08 0 0.15','0.025 0.05 0.30'),
        ('base_link','lift_platform','0 0 -0.006','0.13 0.13 0.012')]:
        el = ET.SubElement(robot.find(f"link[@name='{parent}']"), 'visual', name=name)
        ET.SubElement(el, 'origin', xyz=xyz)
        ET.SubElement(ET.SubElement(el, 'geometry'), 'box', size=size)
        ET.SubElement(ET.SubElement(el, 'material', name='lift_grey'),
                      'color', rgba='0.2 0.3 0.4 1')
    # Preserve the lecture robot's collision meshes, including both fingers.
    for body in robot.findall('link'):
        if body.findall('collision'):
            settings = ET.SubElement(robot, 'gazebo', reference=body.get('name'))
            ET.SubElement(settings, 'selfCollide').text = 'true'
    link = ET.SubElement(robot, 'link', name='rack_support_tray')
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass', value='0.025')
    ET.SubElement(inertial, 'inertia', ixx='0.00001024', iyy='0.00001024',
                  izz='0.00002042', ixy='0', ixz='0', iyz='0')
    for name, xyz, size in [
        ('plate', '0 0 0', '0.07 0.07 0.004'),
        ('mount', '-0.10 0.055 0.035', '0.01 0.01 0.09'),
        ('mount_top', '-0.0725 0.055 0.08', '0.065 0.01 0.008'),
        ('mount_wrist', '-0.045 0.025 0.08', '0.006 0.06 0.008'),
        ('mount_connector', '-0.065 0.055 -0.007', '0.08 0.01 0.01'),
        ('mount_pad', '-0.03 0.0275 -0.007', '0.01 0.055 0.01')]:
        for kind in ('visual','collision'):
            el = ET.SubElement(link, kind, name=name+'_'+kind)
            ET.SubElement(el, 'origin', xyz=xyz, rpy='0 0 0')
            ET.SubElement(ET.SubElement(el, 'geometry'), 'box', size=size)
            if kind == 'visual':
                ET.SubElement(ET.SubElement(el, 'material', name='tray_orange'),
                              'color', rgba='1 0.45 0.05 1')
    joint = ET.SubElement(robot, 'joint', name='rack_tray_mount', type='fixed')
    ET.SubElement(joint, 'parent', link='crane_plus_link4')
    ET.SubElement(joint, 'child', link='rack_support_tray')
    ET.SubElement(joint, 'origin', xyz='0.08 0 0.07', rpy=f'0 {-math.pi/2} 0')
    settings = ET.SubElement(robot, 'gazebo', reference='rack_tray_mount')
    ET.SubElement(settings, 'preserveFixedJoint').text = 'true'
    settings = ET.SubElement(robot, 'gazebo', reference='rack_support_tray')
    ET.SubElement(settings, 'selfCollide').text = 'true'
    return ET.tostring(robot, encoding='unicode')
