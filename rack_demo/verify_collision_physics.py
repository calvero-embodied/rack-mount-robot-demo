"""Isolated headless Gazebo checks; never changes the user's running world."""
import copy
import json
import os
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from demo_scene import HERE
from setup_scene import box
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

reports=[]
for mode in ('fixture','equipment'):
    root=ET.Element('sdf',version='1.7');world=ET.SubElement(root,'world',name='collision_probe')
    for plugin in ('Physics','Contact'):
        ET.SubElement(world,'plugin',filename=f'ignition-gazebo-{plugin.lower()}-system',
                      name='ignition::gazebo::systems::'+plugin)
    physics=ET.SubElement(world,'physics',name='1ms',type='ignored')
    ET.SubElement(physics,'max_step_size').text='.001'
    ET.SubElement(physics,'real_time_factor').text='1'
    robot=copy.deepcopy(ET.parse(HERE/'rack_robot.sdf').getroot().find('model'))
    robot.set('name','probe');world.append(robot)
    assert robot.findtext('self_collide')=='true','Model self collision is disabled'
    for plugin in robot.findall('plugin'): robot.remove(plugin)
    for joint in robot.findall('joint'):
        limit=joint.find('axis/limit')
        if limit is not None:
            limit.find('lower').text='0';limit.find('upper').text='0'
    # Keep the hand joint revolute: its relation to the fixture is non-adjacent.
    tray=robot.find("link[@name='rack_support_tray']")
    for c in list(tray.findall('collision')): tray.remove(c)
    for link in robot.findall('link'):
        for sensor in list(link.findall('sensor')): link.remove(sensor)
    hand=robot.find("link[@name='crane_plus_link_hand']")
    sensor=ET.SubElement(hand,'sensor',name='probe_contact',type='contact')
    ET.SubElement(ET.SubElement(sensor,'contact'),'collision').text=hand.find('collision').get('name')
    if mode=='fixture':
        c=ET.SubElement(tray,'collision',name='deliberate_fixture_collision')
        ET.SubElement(c,'pose').text='.03 -.03 .08 0 0 0'
        ET.SubElement(ET.SubElement(ET.SubElement(c,'geometry'),'box'),'size').text='.04 .04 .04'
    else:
        model=ET.SubElement(world,'model',name='probe_equipment')
        ET.SubElement(model,'static').text='true'
        # Zero arm configuration: wrist z = .0454+.026+.083+.0935.
        ET.SubElement(model,'pose').text='0 -.03 .3479 0 0 0'
        link=ET.SubElement(model,'link',name='chassis')
        box(link,'body','0 0 0','.04 .04 .04','1 0 0 1')
    path=HERE/f'physics_probe_{mode}.sdf';ET.ElementTree(root).write(path,encoding='unicode')
    topics=HERE/f'physics_probe_{mode}_topics.txt'
    topics.write_text('/world/collision_probe/model/probe/link/crane_plus_link_hand/sensor/probe_contact/contact\n')
    output=HERE/f'physics_probe_{mode}.json'
    env=os.environ.copy();env['IGN_PARTITION']='rack_demo_collision_probe_'+mode
    env['IGN_GAZEBO_RESOURCE_PATH']=str(Path(get_package_share_directory('crane_plus_description')).parent)
    if output.exists(): output.unlink()
    monitor=subprocess.Popen([str(HERE/'contact_monitor'),str(topics),str(output)],env=env)
    time.sleep(.5)
    with (HERE/f'physics_probe_{mode}.log').open('w') as log:
        sim=subprocess.Popen(['ign','gazebo','-s','-r',str(path)],env=env,
                             stdout=log,stderr=log,start_new_session=True)
        try:
            end=time.monotonic()+12
            report=None
            while time.monotonic()<end:
                time.sleep(.25)
                if output.exists():
                    report=json.loads(output.read_text())
                    if report.get('pairs'): break
            assert report and report['pairs'],f'No physical hand/{mode} contact: {path}'
            target='rack_support_tray' if mode=='fixture' else 'probe_equipment'
            assert any('crane_plus_link_hand' in p['pair'] and target in p['pair']
                       for p in report['pairs']),report
            reports.append({'mode':mode,'passed':True,**report})
            print(json.dumps(reports[-1]),flush=True)
        finally:
            os.killpg(sim.pid,signal.SIGTERM)
            try: sim.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(sim.pid,signal.SIGKILL);sim.wait()
            monitor.terminate();monitor.wait(timeout=5)
(HERE/'collision_physics_verification.json').write_text(json.dumps(reports,indent=2))
