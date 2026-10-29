#!/opt/pct/bin/python
"""Offline World collision sampling and original PCT GPU tomography.

Only the configured map and its metadata are written. Dynamic pedestrians are
excluded; no World/model files, execution routes or debug exports are generated.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import time
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

import yaml


def collision_boxes(world_file):
    import numpy as np
    from scipy.spatial.transform import Rotation

    def transform(element):
        pose = element.find('pose')
        if pose is not None and pose.get('relative_to'):
            raise ValueError('relative_to requires frame resolution; unsupported in this exporter')
        values = list(map(float, pose.text.split())) if pose is not None else [0.] * 6
        if len(values) != 6 or not np.isfinite(values).all():
            raise ValueError('Invalid SDF pose')
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
        matrix[:3, 3] = values[:3]
        return matrix

    world = ET.parse(world_file).getroot().find('world')
    if world is None:
        raise ValueError('SDF has no world')
    boxes, dynamic = [], []
    for model in world.findall('model'):
        if model.findall('model'):
            raise ValueError('Nested models need explicit pose resolution')
        if any(p.get('filename') == 'libscene_controller.so' for p in model.findall('plugin')):
            dynamic.append(model.get('name'))
            continue
        base = transform(model)
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                size = collision.find('geometry/box/size')
                if size is None:
                    raise ValueError('Unsupported collision shape: ' + model.get('name'))
                dims = np.array(list(map(float, size.text.split())))
                if dims.shape != (3,) or not np.isfinite(dims).all() or (dims <= 0).any():
                    raise ValueError('Invalid collision box: ' + model.get('name'))
                boxes.append((model.get('name'), base @ transform(link) @ transform(collision), dims))
    for include in world.findall('include'):
        if include.findtext('uri') not in ('model://sun', 'model://ground_plane'):
            raise ValueError('External includes must be resolved before map export')
    if not boxes:
        raise ValueError('World has no supported collision boxes')
    return boxes, dynamic


def terrain_regions(boxes):
    """Actual tread/landing solids; rails, lintels and floor slabs are not stairs."""
    import numpy as np
    regions = []
    for name, matrix, size in boxes:
        if re.fullmatch(r'f\d+_stair_(?:south|north)_tread_\d+', name):
            kind = 'stair'
        elif re.fullmatch(r'f\d+_stair_mid_landing', name):
            kind = 'stair_landing'
        else:
            continue
        if not np.allclose(matrix[:3, 2], [0., 0., 1.], atol=1e-6):
            raise ValueError('Terrain region is not horizontal: ' + name)
        regions.append({'name': name, 'kind': kind,
                        'center': matrix[:3, 3].tolist(), 'size': size.tolist(),
                        'yaw': math.atan2(matrix[1, 0], matrix[0, 0])})
    if not regions:
        raise ValueError('No recognized stair/landing regions in configured World')
    return regions


def sample_surfaces(boxes, spacing):
    import numpy as np
    points = []
    for _, matrix, size in boxes:
        for axis in range(3):
            others = [i for i in range(3) if i != axis]
            uv = [np.linspace(-size[i] / 2, size[i] / 2,
                              int(math.ceil(size[i] / spacing)) + 1) for i in others]
            a, b = np.meshgrid(*uv, indexing='ij')
            local = np.zeros((a.size, 3))
            local[:, others[0]], local[:, others[1]] = a.ravel(), b.ravel()
            for sign in (-1, 1):
                local[:, axis] = sign * size[axis] / 2
                points.append((local @ matrix[:3, :3].T + matrix[:3, 3]).astype(np.float32))
    return np.concatenate(points)


def configuration(filename):
    path = Path(filename).resolve()
    config = yaml.safe_load(path.read_text())
    planning, pct, limits = config['planning'], config['pct'], config['constraints']

    def resolve(value):
        if not value:
            raise ValueError('World and map paths must not be empty')
        candidate = Path(value).expanduser()
        return candidate.resolve() if candidate.is_absolute() else (path.parent / candidate).resolve()

    world, output = resolve(pct['world_file']), resolve(planning['map_file'])
    if world == output or world == output.with_suffix('.json'):
        raise ValueError('Map output must not overwrite the source World')
    settings = {key: float(pct['map'][key]) for key in (
        'resolution', 'slice_dh', 'safe_margin', 'interval_free_margin',
        'slope_max', 'standable_ratio', 'cost_barrier')}
    settings.update({key: float(limits[key]) for key in (
        'length', 'width', 'safety_margin', 'max_step', 'clearance')})
    settings['kernel_size'] = int(pct['map']['kernel_size'])
    settings['cost_threshold'] = float(pct['search']['cost_threshold'])
    for key, value in settings.items():
        if not math.isfinite(value) or value < 0 or (
                key not in ('safety_margin', 'safe_margin', 'interval_free_margin') and value == 0):
            raise ValueError('Invalid map parameter: ' + key)
    if settings['kernel_size'] != pct['map']['kernel_size'] or settings['kernel_size'] % 2 != 1:
        raise ValueError('kernel_size must be an odd positive integer')
    if not 0 < settings['standable_ratio'] <= 1 or settings['slope_max'] >= math.pi / 2:
        raise ValueError('Invalid standable_ratio or slope_max')
    if settings['cost_threshold'] >= settings['cost_barrier']:
        raise ValueError('cost_threshold must be below cost_barrier')
    return world, output, settings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, help='global_cfg.yaml')
    args = parser.parse_args()
    world, output, settings = configuration(args.config)
    # 原公式在 threshold=20/barrier=50 时系数为 .6；参数变动时按实际代价轮廓计算。
    radius = math.hypot(settings['length'], settings['width']) / 2 + settings['safety_margin']
    contour = 1. - settings['cost_threshold'] / settings['cost_barrier']
    inflation = max(0., radius - contour * (settings['safe_margin'] + settings['resolution']))
    envelope = inflation + contour * (settings['safe_margin'] + settings['resolution'])
    fingerprint = hashlib.sha256(world.read_bytes() +
        json.dumps(settings, sort_keys=True).encode() +
        Path(__file__).read_bytes() + Path(__file__).with_name('tomogram.py').read_bytes() +
        Path(__file__).with_name('kernels.py').read_bytes()).hexdigest()
    metadata_file = output.with_suffix('.json')
    if output.exists() and metadata_file.exists():
        try:
            metadata = json.loads(metadata_file.read_text())
        except (OSError, ValueError):
            metadata = {}
        if metadata.get('fingerprint') == fingerprint:
            print('PCT map cache:', output, flush=True)
            return

    import numpy as np
    from tomogram import Tomogram
    started = time.perf_counter()
    boxes, dynamic = collision_boxes(world)
    regions = terrain_regions(boxes)
    points = sample_surfaces(boxes, settings['resolution'] * .5)
    minimum, maximum = points.min(axis=0), points.max(axis=0)
    center = (minimum[:2] + maximum[:2]) / 2
    dims = np.ceil((maximum[:2] - minimum[:2]) / settings['resolution']).astype(int) + 4
    ground = 0.0
    slices = int(np.ceil((maximum[2] - ground) / settings['slice_dh']))
    if int(dims.prod()) * slices > 50000000:
        raise ValueError('PCT map exceeds 50 million cells')
    cfg = NS(map=NS(resolution=settings['resolution'], slice_dh=settings['slice_dh'],
                    simplify_layers=False),
             trav=NS(kernel_size=settings['kernel_size'], interval_min=settings['clearance'],
                     interval_free=settings['clearance'] + settings['interval_free_margin'],
                     slope_max=settings['slope_max'], step_max=settings['max_step'] + 1e-6,  # float32 高差容差，header仍记录真实能力
                     standable_ratio=settings['standable_ratio'], cost_barrier=settings['cost_barrier'],
                     safe_margin=settings['safe_margin'], inflation=inflation))
    tomography = Tomogram(cfg)
    tomography.initMappingEnv(center, *dims, slices, ground + settings['slice_dh'])
    print('PCT:', len(boxes), 'collision boxes,', len(points), 'points, grid', dims,
          'slices', slices, flush=True)
    cost, gx, gy, floor, ceiling, timings = tomography.point2map(points)
    cost[~np.isfinite(floor)] = settings['cost_barrier']
    data = np.stack([cost, gx, gy, floor, ceiling]).astype('<f4')
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + '.tmp')
    with temporary.open('wb') as stream:
        stream.write(struct.pack('<8sIII6d', b'PCTMAP02', *floor.shape,
                                 settings['resolution'], float(center[0]), float(center[1]),
                                 settings['max_step'], settings['clearance'], envelope))
        stream.write(data.tobytes(order='C'))
    temporary.replace(output)
    report = {'fingerprint': fingerprint, 'world': str(world),
              'resolution': settings['resolution'], 'shape': list(data.shape),
              'center': center.tolist(), 'point_count': len(points), 'box_count': len(boxes),
              'excluded_dynamic_models': dynamic, 'parameters': settings,
              'regions': regions, 'map_processing_ms': (time.perf_counter() - started) * 1000,
              'gpu_timing_ms': timings, 'upstream': 'byangw/PCT_planner',
              'source': 'SDF collision surface samples, not visual meshes or sensor measurements'}
    metadata_file.write_text(json.dumps(report, indent=2) + '\n')
    print('PCT map written:', output, flush=True)


def observed_surface_map(points, center, dims, ground_min, settings, support, height):
    """Run the original GPU tomography on one explicitly sampled geometry.

    The support mask rejects unknown XY and unrelated upper surfaces. It never
    creates traversability or replaces the original tomography cost/height.
    """
    import numpy as np
    from tomogram import Tomogram
    radius = math.hypot(settings['length'], settings['width']) / 2 + settings['safety_margin']
    contour = 1. - settings['cost_threshold'] / settings['cost_barrier']
    inflation = max(0., radius - contour * (settings['safe_margin'] + settings['resolution']))
    cfg = NS(map=NS(resolution=settings['resolution'], slice_dh=settings['slice_dh'], simplify_layers=False),
             trav=NS(kernel_size=settings['kernel_size'], interval_min=settings['clearance'],
                     interval_free=settings['clearance'] + settings['interval_free_margin'],
                     slope_max=settings['slope_max'], step_max=settings['max_step'] + 1e-6,
                     standable_ratio=settings['standable_ratio'], cost_barrier=settings['cost_barrier'],
                     safe_margin=settings['safe_margin'], inflation=inflation))
    slices = int(np.ceil((points[:, 2].max() - ground_min) / settings['slice_dh']))
    if int(np.prod(dims)) * slices > 50000000:
        raise ValueError('PCT map exceeds 50 million cells')
    tomography = Tomogram(cfg)
    tomography.initMappingEnv(center, *dims, slices, ground_min + settings['slice_dh'])
    cost, gx, gy, floor, ceiling, timing = tomography.point2map(points.astype(np.float32))
    accepted = support[None] & np.isfinite(floor) & (np.abs(floor - height[None]) <= .01)
    cost[~accepted] = settings['cost_barrier']
    floor[~accepted] = np.nan
    return np.stack([cost, gx, gy, floor, ceiling]).astype('<f4'), timing, radius


if __name__ == '__main__':
    main()
