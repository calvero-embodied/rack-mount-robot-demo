"""Convert URDF once and add contact sensors using the converted collision names."""
import subprocess
import xml.etree.ElementTree as ET
from demo_scene import HERE


def write_files(description,lecture_share):
    urdf=HERE/'rack_robot.urdf';urdf.write_text(description)
    sdf=ET.fromstring(subprocess.check_output(['ign','sdf','-p',str(urdf)],text=True))
    # Ignition 6 / DART needs the model flag as well as the link flags.
    ET.SubElement(sdf.find('model'),'self_collide').text='true'
    topics=[]
    for link in sdf.findall('model/link'):
        for index,c in enumerate(link.findall('collision')):
            name=link.get('name')
            if name=='base_link': continue  # Fixed base/table mounting interface.
            # Ignition 6 Contact uses its scoped topic even if <topic> is set.
            topic=f'/world/default/model/crane_plus/link/{name}/sensor/contact_{index}/contact'
            sensor=ET.SubElement(link,'sensor',name=f'contact_{index}',type='contact')
            ET.SubElement(sensor,'always_on').text='true'
            ET.SubElement(sensor,'update_rate').text='50'
            ET.SubElement(sensor,'topic').text=topic
            ET.SubElement(ET.SubElement(sensor,'contact'),'collision').text=c.get('name')
            topics.append(topic)
    ET.indent(sdf)
    ET.ElementTree(sdf).write(HERE/'rack_robot.sdf',encoding='unicode')
    (HERE/'contact_topics.txt').write_text('\n'.join(topics))
    world=ET.parse(lecture_share/'worlds/table.sdf')
    ET.SubElement(world.getroot().find('world'),'plugin',
        filename='ignition-gazebo-contact-system',name='ignition::gazebo::systems::Contact')
    world.write(HERE/'rack_world.sdf',encoding='unicode')
    return str(HERE/'rack_robot.sdf'),str(HERE/'rack_world.sdf')
