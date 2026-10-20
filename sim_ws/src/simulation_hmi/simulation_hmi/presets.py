"""Data and geometry checks for preset placement; no Qt or ROS dependencies."""
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


def load_presets(path):
    scenes = yaml.safe_load(Path(path).read_text())['scenes']
    for name, scene in scenes.items():
        for point in scene['points'].values():
            pose = point['pose']
            if len(pose) != 4 or not all(math.isfinite(x) for x in pose):
                raise ValueError(f'Invalid point in {name}')
        for start, goal in scene['executable_routes']:
            if start not in scene['points'] or goal not in scene['points']:
                raise ValueError(f'Invalid route in {name}')
    return scenes


def task_supported(scene, start, goal):
    return [start, goal] in scene['executable_routes']


def atomic_yaml(path, data):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(yaml.safe_dump(data, allow_unicode=True))
    temporary.replace(path)


def placement_scene(source, name, pose, output):
    """Use live World geometry on storeys; never reuse two_steps bounds there."""
    data = copy.deepcopy(yaml.safe_load(Path(source).read_text()))
    if name == 'three_floor':
        data['scene']['name'] = name
        data['scene']['world_file'] = 'worlds/three_floor.world'
        data['scene']['live_geometry'] = True
    data['scene']['waypoints']['A'] = list(pose)
    atomic_yaml(output, data)


def check_static_placement(world, pose, length=.54, width=.78):
    """Check the oriented wheel footprint plus margin and physical ground support.

    Runtime executor additionally checks actual live collisions, including pedestrians.
    """
    x, y, z, yaw = pose
    hx, hy = length/2+.025, width/2+.025
    support = []
    for model in ET.parse(world).getroot().findall('world/model'):
        if 'pedestrian' in model.get('name', ''):
            continue  # Those poses change; never use an initial pose as a safety guarantee.
        mp = list(map(float, model.findtext('pose', '0 0 0 0 0 0').split()))
        for link in model.findall('link'):
            lp = list(map(float, link.findtext('pose', '0 0 0 0 0 0').split()))
            for collision in link.findall('collision'):
                size = collision.findtext('geometry/box/size')
                if size is None:
                    continue
                cp = list(map(float, collision.findtext('pose', '0 0 0 0 0 0').split()))
                if any(abs(v) > 1e-7 for v in mp[3:5] + lp[3:5] + cp[3:5]):
                    continue  # Presets are on horizontal corridors, not sloping stair members.
                a = mp[5]
                lx = lp[0] + math.cos(lp[5])*cp[0] - math.sin(lp[5])*cp[1]
                ly = lp[1] + math.sin(lp[5])*cp[0] + math.cos(lp[5])*cp[1]
                px = mp[0] + math.cos(a)*lx - math.sin(a)*ly
                py = mp[1] + math.sin(a)*lx + math.cos(a)*ly
                pz = mp[2] + lp[2] + cp[2]
                angle = a + lp[5] + cp[5]
                sx, sy, sz = map(float, size.split())
                def local(xx, yy):
                    return (math.cos(angle)*(xx-px)+math.sin(angle)*(yy-py),
                            -math.sin(angle)*(xx-px)+math.cos(angle)*(yy-py))
                if abs(pz+sz/2-z) < .003:
                    support.append((px, py, angle, sx, sy))
                if pz+sz/2 <= z+.005 or pz-sz/2 >= z+.9:
                    continue
                # Separating axes of the robot and obstacle rectangles.
                overlap = True
                for axis in (yaw, yaw+math.pi/2, angle, angle+math.pi/2):
                    projected = abs((x-px)*math.cos(axis)+(y-py)*math.sin(axis))
                    extent = hx*abs(math.cos(yaw-axis))+hy*abs(math.sin(yaw-axis))
                    extent += sx/2*abs(math.cos(angle-axis))+sy/2*abs(math.sin(angle-axis))
                    if projected >= extent:
                        overlap = False
                        break
                if overlap:
                    raise ValueError(f'点位与静态实体净距不足：{model.get("name")}')
    points = [(x,y)] + [(x+math.cos(yaw)*dx-math.sin(yaw)*dy,
                        y+math.sin(yaw)*dx+math.cos(yaw)*dy)
                       for dx in (-hx,hx) for dy in (-hy,hy)]
    for xx, yy in points:
        if not any(abs(math.cos(a)*(xx-px)+math.sin(a)*(yy-py)) <= sx/2 and
                   abs(-math.sin(a)*(xx-px)+math.cos(a)*(yy-py)) <= sy/2
                   for px, py, a, sx, sy in support):
            raise ValueError('点位缺少完整地面支撑')
