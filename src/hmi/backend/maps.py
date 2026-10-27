"""Read-only conversion of the actual planning cache and Gazebo collision World."""
import math
import struct
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def quaternion(matrix):
    # Stable conversion for the orthonormal SDF rotations, including 180 degrees.
    trace = float(np.trace(matrix))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        return [(matrix[2, 1]-matrix[1, 2])/s, (matrix[0, 2]-matrix[2, 0])/s,
                (matrix[1, 0]-matrix[0, 1])/s, s/4]
    i = int(np.argmax(np.diag(matrix)))
    j, k = (i+1) % 3, (i+2) % 3
    s = math.sqrt(1 + matrix[i, i]-matrix[j, j]-matrix[k, k])*2
    q = [0., 0., 0., 0.]
    q[i], q[j], q[k] = s/4, (matrix[j, i]+matrix[i, j])/s, (matrix[k, i]+matrix[i, k])/s
    q[3] = (matrix[k, j]-matrix[j, k])/s
    return q


def pose(element):
    value = element.find('pose')
    if value is None:
        return np.eye(4)
    if value.attrib:
        raise ValueError('World preview does not support named pose frames')
    values = [float(v) for v in value.text.split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('Invalid World pose')
    result = np.eye(4)
    result[:3, 3], result[:3, :3] = values[:3], rotation(values[3:])
    return result


def world_map(filename):
    world = ET.parse(filename).getroot().find('world')
    if world is None:
        raise ValueError('World has no world element')
    boxes, minimum, maximum = [], np.full(3, np.inf), np.full(3, -np.inf)
    for model in world.findall('model'):
        for link in model.findall('link'):
            base = pose(model) @ pose(link)
            for collision in link.findall('collision'):
                text = collision.findtext('geometry/box/size')
                if text is None:
                    raise ValueError('World preview only supports current box collision map')
                size = np.array([float(v) for v in text.split()])
                transform = base @ pose(collision)
                center, axes = transform[:3, 3], transform[:3, :3]
                extent = np.abs(axes) @ (size/2)
                minimum, maximum = np.minimum(minimum, center-extent), np.maximum(maximum, center+extent)
                boxes.append({'center': center.tolist(), 'size': size.tolist(), 'quaternion': quaternion(axes)})
    if not boxes:
        raise ValueError('World has no collision boxes')
    return {'file': str(filename), 'boxes': boxes, 'bounds': {'min': minimum.tolist(), 'max': maximum.tolist()}}


def global_map(filename):
    filename = Path(filename)
    header = struct.Struct('<8sIII6d')
    with filename.open('rb') as stream:
        data = stream.read(header.size)
    magic, layers, nx, ny, resolution, cx, cy, max_step, clearance, inflation = header.unpack(data)
    if magic != b'PCTMAP02' or min(layers, nx, ny) < 1 or layers*nx*ny > 50000000:
        raise ValueError('Invalid PCT map header')
    expected = header.size+5*layers*nx*ny*4
    if filename.stat().st_size != expected:
        raise ValueError('PCT cache size does not match header')
    values = np.memmap(filename, dtype='<f4', mode='r', offset=header.size, shape=(5, layers, nx, ny))
    stride = max(1, math.ceil(math.sqrt(nx*ny/20000)))
    slices = []
    low_z, high_z = np.inf, -np.inf
    for layer in range(layers):
        ground, cost = values[3, layer, ::stride, ::stride], values[0, layer, ::stride, ::stride]
        # Invalid/missing surfaces are not previewed as fabricated floor points.
        valid = np.isfinite(ground) & np.isfinite(cost) & (ground > -50)
        ix, iy = np.nonzero(valid)
        if not len(ix):
            continue
        z, c = ground[valid], cost[valid]
        x, y = cx+(ix*stride-nx//2)*resolution, cy+(iy*stride-ny//2)*resolution
        height = float(np.median(z))
        low_z, high_z = min(low_z, float(z.min())), max(high_z, float(z.max()))
        points = np.column_stack((x, y, z, c)).round(5).tolist()
        slices.append({'id': layer, 'height': height, 'label': f'切片 {layer} · z≈{height:.2f}m', 'points': points})
    return {'file': str(filename), 'format': 'PCTMAP02', 'resolution': resolution,
            'dimensions': [layers, nx, ny], 'stride': stride,
            'bounds': {'min': [cx-nx//2*resolution, cy-ny//2*resolution, float(low_z)],
                       'max': [cx+(nx-1-nx//2)*resolution, cy+(ny-1-ny//2)*resolution, float(high_z)]},
            'slices': slices}
