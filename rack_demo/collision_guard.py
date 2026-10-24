"""FCL mesh/box collision checks in world coordinates (metres).

Only directly connected robot links and the actual payload/pad contact are
allowed. Hand versus fixture is deliberately checked despite their common wrist.
"""
import itertools
import math
import xml.etree.ElementTree as ET
import numpy as np
import fcl
import trimesh
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from demo_scene import HERE, BASE_Z, SIZES, server_offset, robot_description

MESH_BOUNDS={}


def transform(xyz=(0,0,0), rpy=(0,0,0)):
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();t[:3,3]=xyz
    return t


def origin(node):
    if node is None: return np.eye(4)
    return transform([float(v) for v in node.get('xyz','0 0 0').split()],
                     [float(v) for v in node.get('rpy','0 0 0').split()])


def shape(geometry):
    box=geometry.find('box')
    if box is not None:
        return fcl.Box(*map(float,box.get('size',box.findtext('size','')).split()))
    mesh=geometry.find('mesh')
    filename=mesh.get('filename')
    if filename.startswith('package://'):
        package,relative=filename[10:].split('/',1)
        filename=str(get_package_share_directory(package))+'/'+relative
    m=trimesh.load(filename,force='mesh')
    vertices=np.asarray(m.vertices)*np.array(list(map(float,mesh.get('scale','1 1 1').split())))
    result=fcl.BVHModel();result.beginModel(len(vertices),len(m.faces))
    result.addSubModel(vertices,np.asarray(m.faces,dtype=np.int32));result.endModel()
    MESH_BOUNDS[id(result)]=(vertices.min(0),vertices.max(0))
    return result


def obj(geometry,t):
    class Body:
        def __getattr__(self,name): return getattr(self.collision,name)
    body=Body()
    body.collision=fcl.CollisionObject(geometry,fcl.Transform(t[:3,:3],t[:3,3]))
    if isinstance(geometry,fcl.Box):
        low,high=-np.asarray(geometry.side)/2,np.asarray(geometry.side)/2
    else: low,high=MESH_BOUNDS[id(geometry)]
    center=t[:3,:3]@((low+high)/2)+t[:3,3]
    half=np.abs(t[:3,:3])@((high-low)/2)
    body.low,body.high=center-half,center+half
    return body


class CollisionGuard:
    def __init__(self):
        self.robot=ET.fromstring(robot_description())
        self.joints=list(self.robot.findall('joint'))
        self.parts=[]
        for link in self.robot.findall('link'):
            for c in link.findall('collision'):
                self.parts.append((link.get('name'),c.get('name','mesh'),
                                   shape(c.find('geometry')),origin(c.find('origin'))))
        self.adjacent={frozenset((j.find('parent').get('link'),j.find('child').get('link')))
                       for j in self.joints}
        self.environment=[]
        self.carried=None
        self.support_stock=None
        self.minimum=math.inf

    def fk(self,q,hand):
        values={f'crane_plus_joint{i+1}':v for i,v in enumerate(q[:4])}
        values.update(rack_lift_joint=q[4],crane_plus_joint_hand=hand)
        frames={'world':transform((0,0,BASE_Z))}
        pending=self.joints.copy()
        while pending:
            progress=False
            for j in pending.copy():
                parent=j.find('parent').get('link');child=j.find('child').get('link')
                if parent not in frames: continue
                t=origin(j.find('origin'));value=values.get(j.get('name'),0)
                axis=j.find('axis')
                axis=np.array(list(map(float,axis.get('xyz').split()))) if axis is not None else np.array([0,0,1])
                if j.get('type')=='prismatic': t=t@transform(axis*value)
                elif j.get('type') in ('revolute','continuous'):
                    r=np.eye(4);r[:3,:3]=Rotation.from_rotvec(axis*value).as_matrix();t=t@r
                frames[child]=frames[parent]@t;pending.remove(j);progress=True
            if not progress: raise RuntimeError('Disconnected robot tree')
        return frames

    def refresh(self,models,carried=None,pickup=False):
        self.carried=carried
        self.pickup=pickup
        self.payload_floor=(models[carried]['position']['z'] if carried and pickup else -math.inf)
        self.support_bottom=None
        if self.support_stock and self.support_stock in models:
            size=self.support_stock.removeprefix('rack_stock_')
            self.support_bottom=models[self.support_stock]['position']['z']-SIZES[size]['height']/2
        self.environment=[('Table',fcl.Box(2,2,.05),transform((0,0,BASE_Z-.025)))]
        for filename in ['demo_frame.sdf','stand_1u.sdf','stand_2u.sdf',
                         'rack_stock_1u.sdf','rack_stock_2u.sdf']:
            model=ET.parse(HERE/filename).getroot().find('model');name=model.get('name')
            if name==carried or name not in models: continue
            pose=models[name];p=pose['position'];r=pose.get('orientation',{})
            t=transform([p.get(k,0) for k in ('x','y','z')])
            t[:3,:3]=Rotation.from_quat([r.get(k,1 if k=='w' else 0) for k in ('x','y','z','w')]).as_matrix()
            for link in model.findall('link'):
                for c in link.findall('collision'):
                    offset=list(map(float,c.findtext('pose','0 0 0 0 0 0').split()))
                    self.environment.append((name+'/'+c.get('name'),shape(c.find('geometry')),
                                             t@transform(offset[:3],offset[3:])))

    def check(self,q,hand=.4):
        frames=self.fk(q,hand)
        parts=[(link+'/'+name,link,obj(g,frames[link]@t)) for link,name,g,t in self.parts]
        if self.carried:
            size=self.carried.removeprefix('rack_stock_')
            t=frames['rack_support_tray']@transform((0,0,server_offset(size)))
            t[2,3]=max(t[2,3],self.payload_floor)
            parts.append(('payload', 'payload',obj(fcl.Box(.125,.198,SIZES[size]['height']),t)))
        pairs=[]
        environment=[(name,obj(g,t)) for name,g,t in self.environment]
        for a,b in itertools.combinations(parts,2):
            if a[1]==b[1] or frozenset((a[1],b[1])) in self.adjacent: continue
            if {a[1],b[1]}=={'payload','rack_support_tray'}:
                # Only flat support contact is intended; bracket still checked.
                if 'plate_' in a[0] or 'plate_' in b[0]: continue
            pairs.append((a[0],a[2],b[0],b[2],.002))
        for name,link,body in parts:
            for ename,obstacle in environment:
                if link=='payload' and self.pickup and ename.startswith(self.carried.replace('stock','stand')+'/'):
                    continue  # Equipment initially rests on its own pickup rails.
                # Base is mounted on the table; selected equipment may touch pad.
                if ename=='Table' and link=='crane_plus_base': continue
                if (link=='rack_support_tray' and 'plate_' in name and self.support_stock
                        and ename.startswith(self.support_stock+'/')):
                    plate_top=body.getTranslation()[2]+np.abs(body.getRotation()[2])@np.array([.035,.035,.002])
                    if self.support_bottom is not None and plate_top<=self.support_bottom+.001:
                        continue  # Support contact, at most 1 mm numerical penetration.
                pairs.append((name,body,ename,obstacle,.002))
        for aname,a,bname,b,margin in pairs:
            # An AABB distance is a lower bound on mesh distance. Rejecting a
            # far pair here cannot hide a collision or change the minimum.
            bound=np.linalg.norm(np.maximum(0,np.maximum(a.low-b.high,b.low-a.high)))
            if bound>=max(margin,self.minimum): continue
            result=fcl.CollisionResult()
            if fcl.collide(a.collision,b.collision,fcl.CollisionRequest(),result):
                raise ValueError(f'Collision: {aname} <-> {bname}')
            distance=fcl.distance(a.collision,b.collision,fcl.DistanceRequest(),fcl.DistanceResult())
            self.minimum=min(self.minimum,distance)
            if distance<margin:
                raise ValueError(f'Clearance {distance*1000:.2f} mm: {aname} <-> {bname}')

    def trajectory(self,start,targets,hand=.4):
        # Controller interpolates joint positions, not Cartesian coordinates.
        previous=np.array(start,dtype=float)
        for target in targets:
            target=np.array(target,dtype=float)
            count=max(1,math.ceil(max(np.max(np.abs(target[:4]-previous[:4]))/.01,
                                      abs(target[4]-previous[4])/.001)))
            for i in range(count+1): self.check(previous+(target-previous)*i/count,hand)
            previous=target
