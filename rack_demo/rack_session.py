"""Interactive two-item installation session with pre-motion occupancy checks."""
import argparse
import fcntl
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_demo import Demo
from demo_scene import HERE, BASE_Z, SIZES, LEVELS, joints, target_for, server_offset
from session_scene import (ANGLES, SOURCE_RADIUS, SOURCE_Z, point, snapshot,
    initialize, save, add_fixed_joint, detach, conflict, STATE)
from collision_guard import CollisionGuard

PARK=point(.10,0,.16,.10)


def interpolate(a,b,n=10):
    return [tuple(x+(y-x)*i/n for x,y in zip(a,b)) for i in range(1,n+1)]


class Session(Demo):
    def __init__(self,verbose=False):
        super().__init__('1u','upper')
        self.current=None
        self.motion_count=0
        self.state_data=initialize()
        self.verbose=verbose
        self.guard=CollisionGuard()
        self.collision_payload=None
        self.contact_pickup=False
        self.hand_position=0.0
        self.gripper=ActionClient(self,FollowJointTrajectory,
            '/crane_plus_gripper_controller/follow_joint_trajectory')
        if not self.gripper.wait_for_server(timeout_sec=10):
            raise RuntimeError('Gripper controller is unavailable')

    def grip(self,closed):
        target=-.4 if closed else .4
        self.guard.refresh(snapshot(),self.collision_payload,self.contact_pickup)
        q=self.actual_joints()
        for i in range(41):
            self.guard.check(q,self.hand_position+(target-self.hand_position)*i/40)
        goal=FollowJointTrajectory.Goal()
        goal.trajectory.joint_names=['crane_plus_joint_hand']
        p=JointTrajectoryPoint(positions=[-.4 if closed else .4])
        p.time_from_start.sec=2
        goal.trajectory.points=[p]
        handle=self.wait(self.gripper.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Gripper goal rejected')
        result=self.wait(handle.get_result_async())
        if result.status!=4 or result.result.error_code!=0:
            raise RuntimeError('Gripper motion failed')
        self.hand_position=target
        self.event('gripper_closed' if closed else 'gripper_open')

    def actual_joints(self):
        names=[f'crane_plus_joint{i}' for i in range(1,5)]+['rack_lift_joint']
        end=time.monotonic()+5
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.05)
            if self.state and all(n in self.state for n in names):
                self.hand_position=self.state.get('crane_plus_joint_hand',self.hand_position)
                return [self.state[n] for n in names]
        raise RuntimeError('Joint feedback absent; cannot check trajectory')

    def event(self,state,**details):
        entry={'time':time.time(),'state':state,'size':self.size,
               'level':self.level,**details}
        self.records.append(entry)
        (HERE/'session_log.json').write_text(json.dumps(self.records,ensure_ascii=False,indent=2))
        if self.verbose:
            print(json.dumps(entry,ensure_ascii=False),flush=True)
        else:
            label={'upper':'上段','lower':'下段'}[self.level]
            messages={
                'request_accepted':f"{self.size.upper()}を{label}へ設置します。{'左' if self.size=='1u' else '右'}の機器を取りに行きます。",
                'grasped':f'{self.size.upper()}を把持しました。ラックへ運びます。',
                'holding_waiting_for_fastening':f'{label}で保持しています。',
                'fastened':'ラックへ固定しました。治具を退避します。',
                'complete':f"設置完了（計{len(self.state_data['installed'])}台）。",
            }
            if state in messages:
                print(messages[state],flush=True)

    def move(self,targets,seconds=1):
        names=[f'crane_plus_joint{i}' for i in range(1,5)]+['rack_lift_joint']
        positions=[joints(*t[:3],lift=t[3])+[t[3]] for t in targets]
        self.guard.refresh(snapshot(),self.collision_payload,self.contact_pickup)
        self.guard.minimum=math.inf
        try:
            self.guard.trajectory(self.actual_joints(),positions,self.hand_position)
        except ValueError as exc:
            self.event('path_collision_blocked',reason=str(exc),motion_count=self.motion_count)
            raise RuntimeError(f'衝突する経路のため停止しました: {exc}') from exc
        self.event('path_checked',minimum_clearance_m=self.guard.minimum,
                   samples_max_joint_step_rad=.01,samples_max_lift_step_m=.001)
        goal=FollowJointTrajectory.Goal()
        goal.trajectory.joint_names=names
        for i,t in enumerate(targets,1):
            q=joints(*t[:3],lift=t[3])+[t[3]]
            p=JointTrajectoryPoint(positions=q)
            p.time_from_start.sec=i*seconds
            goal.trajectory.points.append(p)
        handle=self.wait(self.client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Controller rejected the lift/arm trajectory')
        result=self.wait(handle.get_result_async(),len(targets)*seconds+25)
        if result.status!=4 or result.result.error_code!=0:
            raise RuntimeError(str(result))
        expected=joints(*targets[-1][:3],lift=targets[-1][3])+[targets[-1][3]]
        error=lift_error=math.inf
        actual=[]
        end=time.monotonic()+5
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.05)
            if self.state and all(n in self.state for n in names):
                actual=[self.state[n] for n in names]
                error=max(abs(a-b) for a,b in zip(actual[:4],expected[:4]))
                lift_error=abs(actual[4]-expected[4])
                if error<.015 and lift_error<.002:
                    break
        if not self.state or 'rack_lift_joint' not in self.state:
            raise RuntimeError('Lift feedback absent. Restart with the new dedicated launch file.')
        if error>.04 or lift_error>.004:
            self.event('motion_failed',target=list(targets[-1]),actual=actual,
                       expected=expected,arm_error_rad=error,lift_error_m=lift_error)
            raise RuntimeError(f'Motion error: arm={error}, lift={lift_error}')
        self.current=targets[-1]
        self.motion_count+=1
        self.event('motion_complete',target=list(self.current),
                   arm_error_rad=error,lift_error_m=lift_error)

    def line(self,target,n=8):
        self.move(interpolate(self.current,target,n))

    def rotate(self,angle):
        # Stay in front of the rack at a height between the two installation bays.
        start=math.atan2(self.current[1],self.current[0])
        self.move([point(.10,start+(angle-start)*i/10,.16,.10)
                   for i in range(1,11)])

    def check_equipment(self,target):
        pose=snapshot().get(f'rack_stock_{self.size}')
        if not pose:
            raise RuntimeError('Selected equipment disappeared')
        if pose['id']!=self.source_entity:
            raise RuntimeError('Equipment entity changed during pickup/transport')
        p=pose['position']
        expected=(target[0],target[1],BASE_Z+target[2]+server_offset(self.size))
        error=math.dist([p.get(k,0) for k in ('x','y','z')],expected)
        self.event('equipment_check',entity_id=pose['id'],position=p,error_m=error)
        if error>.012:
            raise RuntimeError('Equipment did not follow the gripper / slipped')
        return pose

    def install(self,size,level,automatic=False):
        self.size,self.level=size,level
        self.guard.support_stock=f'rack_stock_{size}'
        self.state_data=json.loads(STATE.read_text())
        if any(v=='carrying' for v in self.state_data['stock'].values()):
            raise RuntimeError('A previous operation stopped while carrying equipment. Reset or restart before continuing.')
        models=snapshot()
        occupied=conflict(self.state_data,size,level,models)
        if occupied:
            label={'upper':'上段','lower':'下段'}[level]
            message=f"{label}には既に{occupied['size'].upper()}が設置されています。ぶつかるため動作しません。"
            print(message,flush=True)
            self.event('collision_blocked',message=message,motion_count=self.motion_count)
            return False
        if self.state_data['stock'][size]!='available':
            print(f'{size.upper()}は既に使用済みです。残っている機器を選んでください。',flush=True)
            self.event('stock_unavailable',motion_count=self.motion_count)
            return False
        angle=ANGLES[size]
        self.source_entity=models[f'rack_stock_{size}']['id']
        source=point(SOURCE_RADIUS,angle,.09,.04)
        raised=point(SOURCE_RADIUS,angle,.115,.04)
        pre=point(.07,angle,.09,.04)
        hold=(*target_for(level),.28 if level=='upper' else 0)
        approach=(.12,0,hold[2],hold[3])
        lowered=(hold[0],0,hold[2]-.008,hold[3])
        pulled=(.09 if level=='upper' else .12,0,lowered[2],lowered[3])
        # Check main waypoints now; move() checks each interpolation batch
        # before sending it to the controller.
        for t in [PARK,source,raised,pre,hold,approach,lowered,pulled]:
            joints(*t[:3],lift=t[3])
        self.event('request_accepted',source_side='left' if size=='1u' else 'right',
                   entity_id=self.source_entity,target=list(hold),decision_source='user_answers_no_ai')
        if self.current is None:
            self.move([PARK],8)
        else:
            self.line(PARK)
        self.grip(False)
        self.rotate(angle)
        self.line(point(.07,angle,.16,.10),5)
        self.line(pre)
        self.line(source)
        # First lift uses real tray contact, with the initially visible object.
        self.collision_payload=f'rack_stock_{size}'
        self.contact_pickup=True
        self.line(raised,4)
        time.sleep(1)
        self.check_equipment(raised)
        self.contact_pickup=False
        self.grip(True)
        topic=f'/rack_demo/grasp/{size}/{int(time.time()*1000)}'
        self.state_data.setdefault('joint_topics',[]).append(topic)
        save(self.state_data)
        add_fixed_joint('crane_plus','rack_support_tray',f'rack_stock_{size}',topic)
        self.event('grasped',method='Gazebo fixed joint after contact pickup')
        self.state_data['stock'][size]='carrying'
        save(self.state_data)
        self.line(point(.10,angle,.16,.10))
        self.check_equipment(self.current)
        self.rotate(0)
        self.line(approach,12)
        self.line(hold,10)
        self.check_equipment(hold)
        self.event('holding_waiting_for_fastening')
        if automatic:
            time.sleep(2)
        else:
            input('ねじ止め完了後にEnter > ')
        # A second physical fixed joint represents completed fastening to the rack.
        mount_topic=f'/rack_demo/mount/{size}/{int(time.time()*1000)}'
        self.state_data['joint_topics'].append(mount_topic)
        save(self.state_data)
        add_fixed_joint('rack_demo_frame','frame',f'rack_stock_{size}',mount_topic)
        detach(topic)
        self.collision_payload=None
        self.grip(False)
        self.event('fastened',method='Gazebo fixed joint to rack')
        self.state_data['stock'][size]='installed'
        self.state_data['installed'].append({'model':f'rack_stock_{size}',
            'size':size,'level':level,'tray_height':hold[2]})
        save(self.state_data)
        self.line(lowered,4)
        time.sleep(.7)
        self.line(pulled,10)
        time.sleep(.7)
        self.check_equipment(hold)
        # Withdraw the complete hand before descending from the upper bay.
        # The lower return rises away, with 4 mm above the bottom crossbeam.
        self.line(PARK,12)
        self.event('complete',installed_count=len(self.state_data['installed']))
        return True


def ask(prompt,choices):
    while True:
        answer=input(prompt).strip().lower()
        if answer in ('q','quit','終了'):
            return None
        if answer in choices:
            return choices[answer]
        print('選択肢を入力してください。終了はqです。')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--prepare',action='store_true',help='Initialize scene only, without arm motion')
    parser.add_argument('--reset',action='store_true',help='Clear installed equipment and refill the two stands')
    parser.add_argument('--sequence',help='Automatic sequence, e.g. 1u:lower,2u:lower,2u:upper')
    args=parser.parse_args()
    sequence=[]
    if args.sequence:
        for entry in args.sequence.split(','):
            size,level=entry.split(':')
            if size not in SIZES or level not in LEVELS:
                parser.error('Invalid size/level')
            sequence.append((size,level))
    lock=open('/tmp/rack_session.lock','w')
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        parser.exit(1,'Another rack session is already running.\n')
    initialize(reset=args.reset)
    if args.prepare:
        print('左: 1U / 右: 2U。機器とラックを初期配置しました。')
        return
    rclpy.init()
    node=Session(verbose=bool(sequence))
    try:
        if sequence:
            report=[]
            for size,level in sequence:
                count=node.motion_count
                result=node.install(size,level,automatic=True)
                report.append({'size':size,'level':level,'installed':result,
                    'motion_batches':node.motion_count-count})
                (HERE/'session_verification.json').write_text(json.dumps(report,indent=2))
        else:
            print('左旋回で1U、右旋回で2U。上段33cm・下段3cm。終了はq。')
            while True:
                status={'available':'待機','installed':'設置済み','carrying':'運搬中'}
                print('在庫：'+ ' / '.join(f'{s.upper()}={status[v]}'
                    for s,v in node.state_data['stock'].items()))
                size=ask('どの機器ですか？ [1u / 2u] > ',{'1u':'1u','1':'1u','2u':'2u','2':'2u'})
                if size is None:
                    break
                level=ask('どこに設置しますか？ [上段 / 下段] > ',
                    {'上段':'upper','上':'upper','upper':'upper','下段':'lower','下':'lower','lower':'lower'})
                if level is None:
                    break
                node.install(size,level)
                print('続けて次の機器を指定できます。')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
