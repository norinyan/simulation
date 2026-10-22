from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
import pytest
from simulation_hmi.presets import check_static_placement, load_presets, placement_scene, task_supported

ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT.parent/'sim_bringup'


def test_all_presets_have_static_clearance_and_support():
    scenes = load_presets(ROOT/'config/presets.yaml')
    assert len(scenes['three_floor']['points']) == 9
    for name, scene in scenes.items():
        for point in scene['points'].values():
            check_static_placement(BRINGUP/'worlds'/f'{name}.world', point['pose'])
    assert task_supported(scenes['two_steps'], 'A', 'B')
    assert not task_supported(scenes['two_steps'], 'B', 'A')
    assert task_supported(scenes['three_floor'], 'F1-ENTRY', 'F3-EAST')


def test_reject_wall_and_unsupported_placement():
    world = BRINGUP/'worlds/three_floor.world'
    with pytest.raises(ValueError):
        check_static_placement(world, [-8.9, 1.9, 0, 0])
    with pytest.raises(ValueError, match='支撑'):
        check_static_placement(world, [0, 1.9, 1.6, 0])


def test_storey_placement_enables_actual_geometry(tmp_path):
    output = tmp_path/'scene.yaml'
    placement_scene(BRINGUP/'config/scene.yaml', 'three_floor', [-5.75, 1.9, 6.4, 0], output)
    scene = yaml.safe_load(output.read_text())['scene']
    assert scene['live_geometry'] is True
    assert scene['name'] == 'three_floor'
    assert scene['waypoints']['start'][2] == 6.4
    assert scene['task_type'] == 'wheel_navigation'
    assert scene['floors'] == [0.0, 3.2, 6.4]
    scenes = load_presets(ROOT/'config/presets.yaml')
    assert task_supported(scenes['three_floor'], 'F1-ENTRY', 'F1-ROOM')
    assert task_supported(scenes['three_floor'], 'F1-ENTRY', 'F2-ROOM')
    assert task_supported(scenes['three_floor'], 'F1-STAIR', 'F2-STAIR')
    assert task_supported(scenes['three_floor'], 'F2-STAIR', 'F1-STAIR')
    points = scenes['three_floor']['points']
    placement_scene(BRINGUP/'config/scene.yaml', 'three_floor', points['F1-STAIR']['pose'], output,
                    goal=points['F2-STAIR']['pose'])
    assert yaml.safe_load(output.read_text())['scene']['task_type'] == 'wheel_navigation'
    assert sum(task_supported(scenes['three_floor'], a, b) for a in points for b in points) == 72


def test_parked_presets_do_not_intersect_configured_pedestrian_sweeps():
    """Do not select a stationary robot spawn on a configured pedestrian route."""
    import math
    scenes = load_presets(ROOT/'config/presets.yaml')
    world = ET.parse(BRINGUP/'worlds/three_floor.world').getroot()
    for model in world.findall('world/model'):
        plugin = model.find("plugin[@name='pedestrian_patrol']")
        if plugin is None:
            continue
        points = [list(map(float, p.findtext('position').split())) for p in plugin.findall('waypoint')]
        for a, b in zip(points, points[1:]):
            angle = math.atan2(b[1]-a[1], b[0]-a[0])
            count = math.ceil(math.dist(a,b)/.02)
            for i in range(count+1):
                px, py = [a[k]+(b[k]-a[k])*i/count for k in range(2)]
                for point in scenes['three_floor']['points'].values():
                    x,y,z,yaw = point['pose']
                    if abs(z-a[2]) > .01:
                        continue
                    separated = False
                    for axis in (yaw,yaw+math.pi/2,angle,angle+math.pi/2):
                        distance = abs((x-px)*math.cos(axis)+(y-py)*math.sin(axis))
                        # Robot's full wheel footprint and pedestrian's physical Box, with 1 cm margin.
                        extent = .27*abs(math.cos(yaw-axis))+.39*abs(math.sin(yaw-axis))
                        extent += .18*abs(math.cos(angle-axis))+.21*abs(math.sin(angle-axis))+.01
                        if distance >= extent:
                            separated = True
                            break
                    assert separated, (point['label'], model.get('name'), i)
