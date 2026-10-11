"""Shared task coordinates, including the supplied upper-right start bay."""
import json
import math
from pathlib import Path

def load_task():
    source=Path(__file__).resolve().parents[1]/'config/patrol_task.json'
    if not source.is_file():
        from ament_index_python.packages import get_package_share_directory
        source=Path(get_package_share_directory('community_demo'))/'config/patrol_task.json'
    task=json.loads(source.read_text())
    if task['start'][:2]!=task['finish'][:2]:
        raise ValueError('The patrol must return to its configured starting coordinate')
    if task['waypoints'][-1]['position']!=task['finish'][:2]:
        raise ValueError('Final waypoint and configured finish disagree')
    return task

TASK=load_task()
START=tuple(TASK['start'])
FINISH=tuple(TASK['finish'])
START_BAY=tuple(TASK['start_bay'])
CORRIDORS=[tuple(r) for r in TASK['drivable_corridors']]

def local_pose(pose,origin=START):
    """Express a field pose in the configured initial robot frame."""
    dx,dy=pose[0]-origin[0],pose[1]-origin[1]
    c,s=math.cos(origin[2]),math.sin(origin[2])
    yaw=pose[2]-origin[2]
    return c*dx+s*dy,-s*dx+c*dy,math.atan2(math.sin(yaw),math.cos(yaw))

def in_start_bay(polygon,margin=.008):
    a,b,c,d=START_BAY
    return bool(polygon) and all(a+margin<=x<=c-margin and b+margin<=y<=d-margin for x,y in polygon)
