"""ROS action-driven rack assistance demo; contact carries the equipment."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from demo_scene import (BASE_Z, LOAD, SIZES, LEVELS, joints, place_server,
                        prepare_rack, remove, server_offset, target_for)

HERE = Path(__file__).resolve().parent


def poses():
    result = subprocess.run(['ign', 'topic', '-e', '-t',
        '/world/default/pose/info', '-n', '1', '--json-output'],
        capture_output=True, text=True, timeout=8, check=True)
    value = json.loads(result.stdout)
    return {p['name']: p for p in value.get('pose', [])}


class Demo(Node):
    def __init__(self, size='1u', level='upper'):
        super().__init__(f'rack_assist_demo_{size}_{level}')
        self.client = ActionClient(self, FollowJointTrajectory,
            '/crane_plus_arm_controller/follow_joint_trajectory')
        self.state = None
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.records = []
        self.size = size
        self.level = level
        self.hold = target_for(level)
        if not self.client.wait_for_server(timeout_sec=15):
            raise RuntimeError('Arm controller is not available')

    def on_state(self, msg):
        self.state = dict(zip(msg.name, msg.position))

    def event(self, state, **details):
        entry = {'time': time.time(), 'state': state, 'size': self.size,
                 'level': self.level, **details}
        self.records.append(entry)
        (HERE/'last_run.json').write_text(json.dumps(self.records, indent=2))
        print(json.dumps(entry), flush=True)

    def wait(self, future, seconds=25):
        rclpy.spin_until_future_complete(self, future, timeout_sec=seconds)
        if not future.done():
            raise RuntimeError('Action timed out; inspect the controller before continuing')
        return future.result()

    def move(self, targets, seconds=5):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f'crane_plus_joint{i}' for i in range(1,5)]
        for index, target in enumerate(targets, 1):
            p = JointTrajectoryPoint()
            p.positions = joints(*target)
            p.time_from_start.sec = index*seconds
            goal.trajectory.points.append(p)
        handle = self.wait(self.client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Controller rejected trajectory')
        result = self.wait(handle.get_result_async(), len(targets)*seconds+15)
        if result.result.error_code != 0 or result.status != 4:
            raise RuntimeError(f'Arm motion failed: {result}')
        expected = joints(*targets[-1])
        actual = []
        error = math.inf
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state is None:
                continue
            actual = [self.state[f'crane_plus_joint{i}'] for i in range(1,5)]
            error = max(abs(a-b) for a,b in zip(expected, actual))
            if error < 0.015:
                break
        if not actual:
            raise RuntimeError('No joint-state feedback received')
        self.event('motion_complete' if error <= 0.035 else 'motion_failed',
                   target=list(targets[-1]), max_joint_error=error,
                   actual=actual, expected=expected)
        if error > 0.035:
            raise RuntimeError('Arm did not reach target')

    def check_server(self, target):
        state = poses().get('rack_demo_server')
        if not state:
            raise RuntimeError('Server not found in Gazebo pose topic')
        position = state['position']
        expected = (target[0], target[1], BASE_Z+target[2]+server_offset(self.size))
        error = math.dist([position.get(k,0) for k in ('x','y','z')], expected)
        self.event('equipment_check', position=position, error_m=error)
        if error > 0.012:
            raise RuntimeError('Equipment slipped or is not on the tray')
        return position

    def run(self, automatic=False):
        hold = self.hold
        # Validate all waypoints before changing the simulator.
        transport = [tuple(a+(b-a)*i/16 for a,b in zip(LOAD,hold))
                     for i in range(1,17)]
        lower = [(hold[0], 0, hold[2]-.02*i/4) for i in range(1,5)]
        retreat = [(hold[0]-.06*i/12, 0, hold[2]-.02) for i in range(1,13)]
        for target in [LOAD, *transport, *lower, *retreat]:
            joints(*target)
        self.event('selection', equipment_height_m=SIZES[self.size]['height'],
                   mass_kg=SIZES[self.size]['mass'], tray_target=list(hold),
                   decision_source='user_selection_no_ai')
        print(f'\nSelected: {self.size.upper()} / {self.level}\n', flush=True)
        remove('rack_demo_server')
        remove('rack_demo_frame')
        self.move([LOAD], 6)
        prepare_rack()
        self.event('ready_to_load')
        if not automatic:
            input('Enter: place the mock equipment on the tray (simulated human loading) > ')
        place_server(LOAD[0], LOAD[1], BASE_Z+LOAD[2]+server_offset(self.size)+.002,
                     size=self.size)
        time.sleep(2)
        self.check_server(LOAD)
        self.event('loaded')
        # Dense IK points keep the tray horizontal during slow translation.
        self.move(transport, 1)
        self.check_server(hold)
        self.event('holding_waiting_for_fastening')
        if automatic:
            time.sleep(3)
        else:
            input('Holding. Human fastens screws. Enter: fastening complete > ')
        position = self.check_server(hold)
        # Explicit demo approximation: replace with static equipment after confirmation.
        # No constraint or teleport is used while the arm carries the equipment.
        orientation = poses()['rack_demo_server'].get('orientation', {'w':1})
        place_server(position.get('x',0), position.get('y',0), position['z'], fixed=True,
                     orientation=orientation, size=self.size)
        self.event('fastened', approximation='static model replaces screw joint')
        self.move(lower, 1)
        self.move(retreat, 1)
        fixed = self.check_server(hold)
        drift = math.dist([fixed.get(k,0) for k in ('x','y','z')],
                          [position.get(k,0) for k in ('x','y','z')])
        self.event('complete', equipment_drift_after_retreat_m=drift)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--auto', action='store_true', help='Automatically confirm human actions')
    parser.add_argument('--size', choices=list(SIZES))
    parser.add_argument('--level', choices=list(LEVELS))
    parser.add_argument('--verify-all', action='store_true', help='Run all four cases automatically')
    args = parser.parse_args()
    if (args.size is None) != (args.level is None):
        parser.error('--size and --level must be specified together')
    if args.verify_all and args.size:
        parser.error('--verify-all cannot be combined with --size/--level')
    cases = [('1u','upper'), ('1u','lower'), ('2u','upper'), ('2u','lower')]
    if not args.verify_all:
        if args.size:
            cases = [(args.size,args.level)]
        elif args.auto:
            cases = [cases[0]]
        else:
            print('1: 1U / upper\n2: 1U / lower\n3: 2U / upper\n4: 2U / lower')
            while True:
                choice = input('Select 1-4 > ').strip()
                if choice in ('1','2','3','4'):
                    cases = [cases[int(choice)-1]]
                    break
                print('Please enter 1, 2, 3 or 4.')
    rclpy.init()
    report = []
    try:
        for size, level in cases:
            node = Demo(size, level)
            try:
                node.run(args.auto or args.verify_all)
                log = HERE/'runs'/f'{size}_{level}.json'
                log.parent.mkdir(exist_ok=True)
                log.write_text(json.dumps(node.records, indent=2))
                report.append({'size':size, 'level':level, 'passed':True,
                               'log':str(log)})
            except Exception as exc:
                report.append({'size':size, 'level':level, 'passed':False, 'error':str(exc)})
                raise
            finally:
                node.destroy_node()
                if args.verify_all:
                    (HERE/'verification.json').write_text(json.dumps(report, indent=2))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
