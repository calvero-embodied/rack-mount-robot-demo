"""Stop only ROS launch trees for the lecture / rack demo simulation."""
import os
from pathlib import Path
import signal
import time

processes = {}
roots = []
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():
        continue
    try:
        args = (proc/'cmdline').read_bytes().replace(b'\0', b' ').decode()
        stat = (proc/'stat').read_text().split(') ',1)[1].split()
        pid = int(proc.name)
        processes[pid] = int(stat[1])
        if args.startswith('ign gazebo '):
            roots.append(pid)
        if '/opt/ros/humble/bin/ros2 launch ' in args and (
            'crane_plus_with_table.launch.py' in args or 'rack_demo.launch.py' in args):
            roots.append(pid)
    except (OSError, UnicodeError):
        pass
selected = set(roots)
while True:
    children = {pid for pid, parent in processes.items() if parent in selected}
    if children <= selected:
        break
    selected |= children
for pid in sorted(selected, reverse=True):
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
print('Stopped simulation processes:', sorted(selected))
time.sleep(2)
