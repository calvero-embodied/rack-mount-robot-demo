"""Add a tabletop rack mock-up to the running lecture Gazebo world."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent


def box(link, name, xyz, size, color, collision=True):
    for kind in (['visual', 'collision'] if collision else ['visual']):
        item = ET.SubElement(link, kind, name=f'{name}_{kind}')
        ET.SubElement(item, 'pose').text = f'{xyz} 0 0 0'
        geometry = ET.SubElement(item, 'geometry')
        ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = size
        if kind == 'visual':
            material = ET.SubElement(item, 'material')
            ET.SubElement(material, 'ambient').text = color
            ET.SubElement(material, 'diffuse').text = color


def rack():
    root = ET.Element('sdf', version='1.6')
    model = ET.SubElement(root, 'model', name='rack_demo_frame')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = '0.30 0.16 1.015 0 0 0'
    link = ET.SubElement(model, 'link', name='frame')
    steel = '0.12 0.16 0.22 1'
    for x in (-0.075, 0.075):
        for y in (-0.106, 0.106):
            box(link, f'post_{x}_{y}', f'{x} {y} 0.12', '0.012 0.012 0.24', steel)
    for z in (0.006, 0.234):
        for y in (-0.106, 0.106):
            box(link, f'cross_{z}_{y}', f'0 {y} {z}', '0.16 0.012 0.012', steel)
        for x in (-0.075, 0.075):
            box(link, f'beam_{z}_{x}', f'{x} 0 {z}', '0.012 0.224 0.012', steel)
    # Visual markers only: these represent mounting locations, not screw threads.
    for level in range(1, 10):
        for y in (-0.106, 0.106):
            box(link, f'mount_{level}_{y}', f'-0.082 {y} {level * 0.022}',
                '0.002 0.006 0.004', '0.9 0.65 0.1 1', False)
    return root


def server():
    root = ET.Element('sdf', version='1.6')
    model = ET.SubElement(root, 'model', name='rack_demo_server')
    ET.SubElement(model, 'pose').text = '0.17 -0.18 1.035 0 0 0'
    link = ET.SubElement(model, 'link', name='chassis')
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass').text = '0.05'
    inertia = ET.SubElement(inertial, 'inertia')
    for key, value in {'ixx': 0.0001521, 'iyy': 0.00006135,
                       'izz': 0.0002104, 'ixy': 0, 'ixz': 0, 'iyz': 0}.items():
        ET.SubElement(inertia, key).text = str(value)
    box(link, 'body', '0 0 0', '0.12 0.19 0.018', '0.55 0.62 0.7 1')
    box(link, 'face', '-0.061 0 0', '0.003 0.202 0.018', '0.05 0.25 0.48 1')
    for index in range(8):
        box(link, f'port_{index}', f'-0.063 {-0.074 + index * 0.021} 0',
            '0.001 0.013 0.008', '0.015 0.02 0.025 1', False)
    return root


def main():
    for name, root in [('frame', rack()), ('server', server())]:
        path = HERE / f'{name}.sdf'
        ET.indent(root)
        ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
        request = f'sdf_filename: "{path.as_posix()}", allow_renaming: false'
        result = subprocess.run([
            'ign', 'service', '-s', '/world/default/create',
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean', '--timeout', '5000',
            '--req', request], capture_output=True, text=True, check=True)
        print(f'{name}: {result.stdout.strip()}')
        if 'data: true' not in result.stdout:
            raise RuntimeError(f'Could not create {name}: {result.stderr}')
    scene = subprocess.run([
        'ign', 'service', '-s', '/world/default/scene/graph',
        '--reqtype', 'ignition.msgs.Empty', '--reptype', 'ignition.msgs.StringMsg',
        '--timeout', '5000', '--req', ' '],
        capture_output=True, text=True, check=True)
    for model in ('rack_demo_frame', 'rack_demo_server'):
        if model not in scene.stdout:
            raise RuntimeError(f'Model absent from scene graph: {model}')
    print('Verified: rack and server are present in the Gazebo scene graph.')


if __name__ == '__main__':
    main()
