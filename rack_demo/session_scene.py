"""Persistent left/right stock, pickup stands and fixed-joint grasp helpers."""
import json
import math
import subprocess
import shlex
import time
import xml.etree.ElementTree as ET
from demo_scene import (HERE, BASE_Z, SIZES, LEVELS, call, create, remove,
                        place_server, prepare_rack, server_offset)
from setup_scene import box

STATE = HERE/'session_state.json'
SOURCE_RADIUS = .21
SOURCE_Z = .102  # Bottom of equipment on the pickup stands.
ANGLES = {'1u': math.pi/2, '2u': -math.pi/2}


def point(radius, angle, z, lift=0):
    return (radius*math.cos(angle), radius*math.sin(angle), z, lift)


def snapshot():
    result = subprocess.run(['ign','topic','-e','-t','/world/default/pose/info',
        '-n','1','--json-output'], capture_output=True,text=True,timeout=10,check=True)
    value=json.JSONDecoder().raw_decode(result.stdout.lstrip())[0]
    return {p['name']:p for p in value.get('pose',[])}


def save(state):
    temporary = STATE.with_suffix('.tmp')
    temporary.write_text(json.dumps(state,ensure_ascii=False,indent=2))
    temporary.replace(STATE)


def initialize(reset=False):
    models = snapshot()
    if reset and STATE.exists() and 'rack_demo_frame' in models:
        previous=json.loads(STATE.read_text())
        for topic in previous.get('joint_topics',[]):
            detach(topic,confirm=False)
    if STATE.exists() and 'rack_demo_frame' in models and not reset:
        state=json.loads(STATE.read_text())
        if all(f'rack_stock_{s}' in models for s in SIZES):
            return state
        raise RuntimeError('Session objects are missing. Inspect the scene, then use --reset.')
    for name in ['rack_demo_server','rack_demo_frame','rack_stock_1u','rack_stock_2u',
                 'rack_stand_1u','rack_stand_2u','wood_cube_5cm']:
        remove(name)
    prepare_rack()
    for size, angle in ANGLES.items():
        x,y,_,_=point(SOURCE_RADIUS,angle,0)
        root=ET.Element('sdf',version='1.6')
        model=ET.SubElement(root,'model',name=f'rack_stand_{size}')
        ET.SubElement(model,'static').text='true'
        ET.SubElement(model,'pose').text=f'{x} {y} {BASE_Z} 0 0 {angle}'
        link=ET.SubElement(model,'link',name='stand')
        for side in (-.085,.085):
            box(link,f'rail_{side}',f'0 {side} .092','.14 .016 .02','0.25 0.3 0.35 1')
            for end in (-.05,.05):
                box(link,f'leg_{side}_{end}',f'{end} {side} .043',
                    '.016 .016 .086','0.25 0.3 0.35 1')
        create(root,f'stand_{size}.sdf')
        place_server(x,y,BASE_Z+SOURCE_Z+SIZES[size]['height']/2+.001,
            size=size,name=f'rack_stock_{size}',orientation={
                'w':math.cos(angle/2),'z':math.sin(angle/2)})
    state={'version':2,'stock':{s:'available' for s in SIZES},'installed':[], 'joint_topics':[]}
    save(state)
    return state


def add_fixed_joint(parent, parent_link, child, topic):
    entity_id=snapshot()[parent]['id']
    xml=(f'<parent_link>{parent_link}</parent_link><child_model>{child}</child_model>'
         f'<child_link>chassis</child_link><detach_topic>{topic}</detach_topic>'
         f'<attach_topic>{topic}/attach</attach_topic><output_topic>{topic}/state</output_topic>')
    request=(f'entity: {{id: {entity_id}, name: "{parent}", type: MODEL}}, plugins: {{'
        'name: "ignition::gazebo::systems::DetachableJoint", '
        'filename: "libignition-gazebo-detachable-joint-system.so", '
        f'innerxml: {json.dumps(xml)}}}')
    listener=subprocess.Popen(['ign','topic','-e','-t',topic+'/state','-n','1',
        '--json-output'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:
        time.sleep(.3)
        call('/world/default/entity/system/add','EntityPlugin_V',request)
        try:
            output,_=listener.communicate(timeout=8)
        except subprocess.TimeoutExpired:
            # A newly advertised one-shot state may precede topic discovery.
            # The object is still supported by the pad (or by its grasp joint).
            listener.kill();listener.communicate()
            send_empty(topic)
            time.sleep(.5)
            listener=subprocess.Popen(['ign','topic','-e','-t',topic+'/state',
                '-n','1','--json-output'],stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,text=True)
            time.sleep(.7)
            send_empty(topic+'/attach')
            output,_=listener.communicate(timeout=10)
        if json.JSONDecoder().raw_decode(output.lstrip())[0].get('data')!='attached':
            raise RuntimeError('Fixed-joint plugin did not confirm attachment')
    finally:
        if listener.poll() is None:
            listener.kill()
            listener.communicate()


def send_empty(topic):
    binary=HERE/'publish_empty'
    source=HERE/'publish_empty.cc'
    if not binary.exists() or binary.stat().st_mtime<source.stat().st_mtime:
        flags=subprocess.check_output(['pkg-config','--cflags','--libs',
            'ignition-transport11'],text=True)
        subprocess.run(['g++',str(source),'-o',str(binary),*shlex.split(flags)],check=True)
    subprocess.run([str(binary),topic],check=True,capture_output=True,text=True,timeout=8)


def detach(topic,confirm=True):
    listener=None
    if confirm:
        listener=subprocess.Popen(['ign','topic','-e','-t',topic+'/state','-n','1',
            '--json-output'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        time.sleep(.7)
    try:
        send_empty(topic)
        if listener:
            output,_=listener.communicate(timeout=8)
            if json.JSONDecoder().raw_decode(output.lstrip())[0].get('data')!='detached':
                raise RuntimeError('Gripper release was not confirmed')
        time.sleep(.3)
    finally:
        if listener and listener.poll() is None:
            listener.kill(); listener.communicate()


def conflict(state,size,level,models):
    bottom=BASE_Z+LEVELS[level]+.002
    top=bottom+SIZES[size]['height']
    for installed in state['installed']:
        pose=models.get(installed['model'])
        if not pose:
            raise RuntimeError('Installed equipment is missing; refusing to use stale occupancy data')
        p=pose['position']; h=SIZES[installed['size']]['height']
        # Conservative AABB with a 5mm margin; all installed equipment faces forward.
        if (abs(p.get('x',0)-.23)<.13 and abs(p.get('y',0))<.203 and
            bottom < p['z']+h/2+.005 and top > p['z']-h/2-.005):
            return installed
    return None
