"""Check all four pickup/install/withdraw paths without moving the robot."""
import copy
import json
import math
import xml.etree.ElementTree as ET
from collision_guard import CollisionGuard
from demo_scene import HERE,BASE_Z,SIZES,joints,server_offset
from session_scene import point

g=CollisionGuard()
models={}
for file in ['demo_frame.sdf','stand_1u.sdf','stand_2u.sdf','rack_stock_1u.sdf','rack_stock_2u.sdf']:
    m=ET.parse(HERE/file).getroot().find('model')
    p=list(map(float,m.findtext('pose').split()))
    models[m.get('name')]={'position':dict(zip(('x','y','z'),p[:3])),
        'orientation':{'w':math.cos(p[5]/2),'z':math.sin(p[5]/2)}}
initial=copy.deepcopy(models)
park=(.1,0,.16,.1)
def ik(t): return joints(*t[:3],lift=t[3])+[t[3]]
report=[]
for order in [('1u','lower','2u','upper'),('2u','upper','1u','lower'),
              ('1u','upper','2u','lower'),('2u','lower','1u','upper')]:
    models=copy.deepcopy(initial);current=park;hand=.4
    for size,level in (order[:2],order[2:]):
        stock=f'rack_stock_{size}';g.support_stock=stock;g.minimum=math.inf
        angle=math.pi/2 if size=='1u' else -math.pi/2
        payload=None;pickup=False
        def line(target,n=8):
            global current
            targets=[tuple(a+(b-a)*i/n for a,b in zip(current,target)) for i in range(1,n+1)]
            g.refresh(models,payload,pickup);g.trajectory(ik(current),[ik(t) for t in targets],hand)
            current=target
        def rotate(end):
            global current
            start=math.atan2(current[1],current[0])
            targets=[point(.1,start+(end-start)*i/10,.16,.1) for i in range(1,11)]
            g.refresh(models,payload,pickup);g.trajectory(ik(current),[ik(t) for t in targets],hand)
            current=targets[-1]
        line(park);rotate(angle);line(point(.07,angle,.16,.1),5)
        line(point(.07,angle,.09,.04));line(point(.21,angle,.09,.04))
        payload=stock;pickup=True;line(point(.21,angle,.115,.04),4);pickup=False
        # The supported equipment has now physically risen to the pad surface.
        models[stock]['position']['z']=BASE_Z+.115+server_offset(size)
        for i in range(41): g.check(ik(current),.4-.8*i/40)
        hand=-.4
        line(point(.1,angle,.16,.1));rotate(0)
        hold=(.23,0,.33 if level=='upper' else .03,.28 if level=='upper' else 0)
        line((.12,0,hold[2],hold[3]),12);line(hold,10)
        models[stock]={'position':dict(zip(('x','y','z'),(.23,0,BASE_Z+hold[2]+server_offset(size)))),
                       'orientation':{'w':1}}
        payload=None;g.refresh(models)
        for i in range(41): g.check(ik(current),-.4+.8*i/40)
        hand=.4
        line((.23,0,hold[2]-.008,hold[3]),4)
        line((.09 if level=='upper' else .12,0,hold[2]-.008,hold[3]),10)
        line(park,12)
        entry={'order':list(order),'size':size,'level':level,'passed':True,
               'minimum_clearance_m':g.minimum}
        report.append(entry);print(json.dumps(entry),flush=True)
(HERE/'collision_paths_verification.json').write_text(json.dumps(report,indent=2))
