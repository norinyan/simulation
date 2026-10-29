#!/usr/bin/env python3
"""Generate the observed PCD scene; no hole filling, smoothing or alignment."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import time
from importlib.machinery import SourceFileLoader
import numpy as np
from scipy.spatial import cKDTree, Delaunay
from scipy.ndimage import distance_transform_edt
import yaml


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_pcd(path):
    blob = Path(path).read_bytes()
    header, payload = blob.split(b'DATA binary\n', 1)
    fields = {line.split()[0]: line.split()[1:] for line in header.decode().splitlines()
              if line and not line.startswith('#')}
    for key, value in [('FIELDS', ['x', 'y', 'z', 'intensity']), ('SIZE', ['4']*4),
                       ('TYPE', ['F']*4), ('COUNT', ['1']*4), ('HEIGHT', ['1'])]:
        if fields.get(key) != value:
            raise ValueError('Unsupported PCD ' + key)
    points = np.frombuffer(payload, '<f4').reshape(-1, 4)[:, :3].copy()
    if len(points) != int(fields['POINTS'][0]) or not np.isfinite(points).all():
        raise ValueError('Invalid PCD count or coordinates')
    return points


def surfaces(points):
    selected = []
    for i, neighbors in enumerate(cKDTree(points).query_ball_point(points, .35)):
        if len(neighbors) < 6 or points[i, 2] >= -.5:
            continue
        values, vectors = np.linalg.eigh(np.cov(points[neighbors].astype(float).T, bias=True))
        if values[1] >= .0001 and values[0]/values[1] < .1 and abs(vectors[2, 0]) > .95:
            selected.append(i)
    vertices = points[selected]
    _, unique = np.unique(vertices[:, :2], axis=0, return_index=True)
    vertices = vertices[unique]
    triangulation = Delaunay(vertices[:, :2])
    q = vertices[triangulation.simplices]
    edge = np.stack([q[:, 1]-q[:, 0], q[:, 2]-q[:, 1], q[:, 0]-q[:, 2]], 1)
    normal = np.cross(q[:, 1]-q[:, 0], q[:, 2]-q[:, 0])
    norm = np.linalg.norm(normal, axis=1)
    accepted = ((np.linalg.norm(edge, axis=2).max(1) <= .3)
                & (abs(normal[:, 2]) >= np.cos(.4)*norm) & (norm > 2e-7))
    return vertices, triangulation, accepted


def sample_triangles(vertices, faces, spacing):
    samples = []
    for q in vertices[faces]:
        # A barycentric lattice with longest edge/n <= spacing has covering
        # radius <= spacing. No triangle extends beyond its measured vertices.
        n = max(1, int(np.ceil(max(np.linalg.norm(q[1]-q[0]), np.linalg.norm(q[2]-q[0]),
                                  np.linalg.norm(q[2]-q[1]))/spacing)))
        i, j = np.triu_indices(n+1)
        a, b = i/n, (j-i)/n
        samples.append(q[0] + a[:, None]*(q[1]-q[0]) + b[:, None]*(q[2]-q[0]))
    return np.concatenate(samples).astype(np.float32)


def voxel_mesh(indices, resolution):
    offsets = np.array([[0,0,0], [1,0,0], [1,1,0], [0,1,0],
                        [0,0,1], [1,0,1], [1,1,1], [0,1,1]])
    vertices = ((indices[:, None]+offsets)*resolution).reshape(-1, 3)
    quads = [[0,3,2,1], [4,5,6,7], [0,1,5,4], [1,2,6,5], [2,3,7,6], [3,0,4,7]]
    faces = np.array([[q[0],q[1],q[2]] for q in quads]+[[q[0],q[2],q[3]] for q in quads])
    return vertices, (faces[None]+np.arange(len(indices))[:, None, None]*8).reshape(-1,3)


def display_classification(data, ground, support, distance, origin, resolution, settings):
    """Exact shared-grid queries at ground + .44m; never project arbitrary Z."""
    pct_ground, cost, ceiling = data[3], data[0], data[4]
    pct = np.any(np.isfinite(pct_ground) & np.isfinite(cost)
        & (cost <= settings['cost_threshold'])
        & (abs(pct_ground-ground[None]) <= .01)
        & (~np.isfinite(ceiling) | (ceiling-pct_ground >= settings['clearance']-1e-6)), axis=0)
    zindex = (np.where(support, ground, origin[2])+.44-origin[2])/resolution-.5
    k = np.floor(zindex).astype(int); fraction = zindex-k
    inside = support & (k >= 0) & (k+1 < distance.shape[2])
    k = np.clip(k, 0, distance.shape[2]-2)
    x, y = np.indices(ground.shape)
    clearance = (1-fraction)*distance[x,y,k]+fraction*distance[x,y,k+1]
    clearance[~inside] = -np.inf
    category = np.zeros(ground.shape, dtype=np.uint8)  # 0 unknown, 1 blocked, 2 pass
    category[support] = 1
    category[support & pct & (clearance >= .05)] = 2
    return category, clearance, pct


def display_meshes(vertices, faces, ground, support, category, origin, resolution):
    """Clip observed triangles to classified cells; no square fills unknown gaps."""
    pieces = {1: [], 2: []}
    def clip(poly, axis, bound, keep_greater):
        result = []
        for previous, current in zip(poly, np.roll(poly, -1, axis=0)):
            a = (previous[axis] >= bound) if keep_greater else (previous[axis] <= bound)
            b = (current[axis] >= bound) if keep_greater else (current[axis] <= bound)
            if a: result.append(previous)
            if a != b:
                t = (bound-previous[axis])/(current[axis]-previous[axis])
                result.append(previous+t*(current-previous))
        return np.asarray(result)
    for triangle in vertices[faces]:
        lo = np.maximum(0, np.floor((triangle[:,:2].min(0)-origin[:2])/resolution).astype(int))
        hi = np.minimum(np.array(ground.shape)-1, np.floor((triangle[:,:2].max(0)-origin[:2])/resolution).astype(int))
        for i in range(lo[0],hi[0]+1):
            for j in range(lo[1],hi[1]+1):
                value = int(category[i,j])
                if not value: continue
                poly = triangle.copy()
                for axis, cell in enumerate([i,j]):
                    for bound, greater in [(origin[axis]+cell*resolution, True),
                                           (origin[axis]+(cell+1)*resolution, False)]:
                        poly = clip(poly,axis,bound,greater)
                        if len(poly)<3: break
                    if len(poly)<3: break
                if len(poly)<3: continue
                # Ground queries include the measured source voxel top; keep
                # color above that same surface so it does not z-fight or hide.
                poly[:,2] = np.maximum(poly[:,2],ground[i,j])+.008
                for n in range(1,len(poly)-1):
                    piece = poly[[0,n,n+1]]
                    if np.linalg.norm(np.cross(piece[1]-piece[0],piece[2]-piece[0]))>1e-10:
                        pieces[value].append(piece)
    result = {}
    for value, name in [(1,'blocked'),(2,'passable')]:
        if pieces[value]:
            v = np.concatenate(pieces[value])
            result['display_'+name] = (v,np.arange(len(v)).reshape(-1,3),DISPLAY_COLORS[name])
    return result


DISPLAY_COLORS = {'passable': (.12, .78, .36, 1.),
                  'blocked': (.95, .27, .12, 1.), 'legend': (.92, .95, 1., 1.)}


def legend_mesh(origin):
    """Embed complete monochrome glyphs as pixel runs in the existing DAE."""
    from matplotlib.font_manager import FontProperties, findfont
    from PIL import Image, ImageDraw, ImageFont
    font = ImageFont.truetype(findfont(FontProperties(family='DejaVu Sans')), 32)
    reference = font.getbbox('GREEN')
    scale = .55/(reference[3]-reference[1])
    vertices, faces = [], []
    lines = ['GREEN  PCT + SCAN PASS', 'RED  SUPPORTED / BLOCKED',
             'VOID  UNKNOWN / NO FLOOR', 'COLOR IS NOT CONNECTIVITY',
             'COMPARE: DELETE display_overlay']
    for row, text in enumerate(lines):
        left, top, right, bottom = font.getbbox(text)
        bitmap = Image.new('1', (right-left, bottom-top))
        ImageDraw.Draw(bitmap).text((-left, -top), text, fill=1, font=font)
        # Contiguous lit pixels become rectangles. This preserves every glyph
        # contour and hole without unconstrained triangulation across letters.
        for y, pixels in enumerate(np.asarray(bitmap, dtype=bool)):
            changes = np.diff(np.r_[False,pixels,False].astype(np.int8))
            for x0, x1 in zip(np.flatnonzero(changes==1), np.flatnonzero(changes==-1)):
                xlo, xhi = origin[0]+x0*scale, origin[0]+x1*scale
                yhi = origin[1]-.85*row+(bottom-top-y)*scale
                ylo = yhi-scale
                n = len(vertices)
                vertices.extend([[xlo,ylo,origin[2]],[xhi,ylo,origin[2]],
                                 [xhi,yhi,origin[2]],[xlo,yhi,origin[2]]])
                faces.extend([[n,n+1,n+2],[n,n+2,n+3]])
    return np.asarray(vertices), np.asarray(faces)


def write_dae(path, vertices, faces, overlays=None):
    """Named submeshes keep display-only geometry out of Gazebo collision."""
    geometry = {'original': (vertices, faces, (.65, .7, .75, 1.))}
    geometry.update(overlays or {})
    with path.open('w') as f:
        f.write('<?xml version="1.0"?><COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1"><asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>')
        f.write('<library_effects>')
        for name, (_, _, color) in geometry.items():
            rgba = ' '.join(map(str, color))
            # A single emissive contribution gives categorical colors exactly;
            # the observed geometry keeps ordinary, subdued scene lighting.
            emission = '0 0 0 1' if name == 'original' else rgba
            ambient = ' '.join(map(str, [x*.25 for x in color[:3]]+[1.])) if name == 'original' else '0 0 0 1'
            diffuse = ' '.join(map(str, [x*.55 for x in color[:3]]+[1.])) if name == 'original' else '0 0 0 1'
            f.write(f'<effect id="{name}-effect"><profile_COMMON><technique sid="common"><lambert><emission><color>{emission}</color></emission><ambient><color>{ambient}</color></ambient><diffuse><color>{diffuse}</color></diffuse></lambert></technique></profile_COMMON></effect>')
        f.write('</library_effects><library_materials>')
        for name in geometry:
            f.write(f'<material id="{name}-material"><instance_effect url="#{name}-effect"/></material>')
        f.write('</library_materials><library_geometries>')
        for name, (v, triangles, _) in geometry.items():
            f.write(f'<geometry id="{name}-geometry" name="{name}"><mesh><source id="{name}-positions"><float_array id="{name}-data" count="{v.size}">')
            for chunk in np.array_split(v, max(1, len(v)//10000)):
                f.write(' '.join('%.9g' % x for x in chunk.ravel())+' ')
            f.write(f'</float_array><technique_common><accessor source="#{name}-data" count="{len(v)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source><vertices id="{name}-vertices"><input semantic="POSITION" source="#{name}-positions"/></vertices><triangles material="{name}-material" count="{len(triangles)}"><input semantic="VERTEX" source="#{name}-vertices" offset="0"/><p>')
            for chunk in np.array_split(triangles, max(1,len(triangles)//10000)):
                f.write(' '.join(map(str,chunk.ravel()))+' ')
            f.write('</p></triangles></mesh></geometry>')
        f.write('</library_geometries><library_visual_scenes><visual_scene id="scene">')
        for name in geometry:
            f.write(f'<node id="{name}-node" name="{name}"><instance_geometry url="#{name}-geometry"><bind_material><technique_common><instance_material symbol="{name}-material" target="#{name}-material"/></technique_common></bind_material></instance_geometry></node>')
        f.write('</visual_scene></library_visual_scenes><scene><instance_visual_scene url="#scene"/></scene></COLLADA>')


def write_world(path, minimum, maximum, overlay_names):
    import xml.etree.ElementTree as ET
    root = ET.Element('sdf', version='1.6'); world = ET.SubElement(root, 'world', name='pcd_scene')
    ET.SubElement(world, 'gravity').text = '0 0 -9.81'
    physics = ET.SubElement(world, 'physics', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '0.005'
    ET.SubElement(physics, 'real_time_update_rate').text = '200'
    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '0.7 0.7 0.7 1'
    ET.SubElement(scene, 'background').text = '0.10 0.13 0.17 1'
    ET.SubElement(scene, 'grid').text = 'false'
    ET.SubElement(ET.SubElement(world, 'light', name='sun', type='directional'), 'direction').text = '-0.5 -0.5 -1'
    center = (minimum+maximum)/2
    span = max(maximum[:2]-minimum[:2])
    camera = ET.SubElement(ET.SubElement(world, 'gui'), 'camera', name='overview')
    ET.SubElement(camera, 'pose').text = f'{center[0]} {center[1]-span*.75} {center[2]+span*.95} 0 .90 1.57079632679'
    ET.SubElement(camera, 'view_controller').text = 'orbit'
    def mesh(parent, name):
        g = ET.SubElement(ET.SubElement(parent, 'geometry'), 'mesh')
        ET.SubElement(g, 'uri').text = 'map.dae'
        sub = ET.SubElement(g, 'submesh'); ET.SubElement(sub, 'name').text = name
        ET.SubElement(sub, 'center').text = 'false'
    model = ET.SubElement(world, 'model', name='observed_map')
    ET.SubElement(model, 'static').text = 'true'; link = ET.SubElement(model, 'link', name='map')
    mesh(ET.SubElement(link, 'collision', name='map'), 'original')
    mesh(ET.SubElement(link, 'visual', name='map'), 'original')
    overlay = ET.SubElement(world, 'model', name='display_overlay')
    ET.SubElement(overlay, 'static').text = 'true'; link = ET.SubElement(overlay, 'link', name='display')
    for name in overlay_names:
        visual = ET.SubElement(link, 'visual', name=name)
        mesh(visual, name); ET.SubElement(visual, 'cast_shadows').text = 'false'
        color = DISPLAY_COLORS[name.removeprefix('display_')]
        material = ET.SubElement(visual, 'material')
        for key in ['ambient', 'diffuse', 'emissive']:
            ET.SubElement(material, key).text = ' '.join(map(str, color)) if key == 'emissive' else '0 0 0 1'
    ET.indent(root)
    path.write_text(ET.tostring(root, encoding='unicode')+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--config', type=Path)
    args = parser.parse_args()
    source_config = Path(__file__).resolve().parents[2]/'global_planning/config/global_cfg.yaml'
    if args.config is None:
        if source_config.exists():
            args.config = source_config
        else:
            from ament_index_python.packages import get_package_share_directory
            args.config = Path(get_package_share_directory('global_planning'))/'config/global_cfg.yaml'
    if args.output is None:
        if source_config.exists():
            args.output = Path(__file__).resolve().parents[1]/'maps'
        else:
            parser.error('Installed generator requires --output pointing to a writable directory')
    started = time.monotonic()
    source = Path(args.source)
    points = load_pcd(source)
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=True)
    # A failed regeneration must not leave an old manifest authorizing a mixed
    # generation. The manifest is published only after every output validates.
    (out/'map.yaml').unlink(missing_ok=True)
    r = .05
    config = yaml.safe_load(args.config.read_text())
    # These are the existing runtime contracts, also used by the geometry
    # conversion below. Reject changed configs rather than mislabeling safety.
    control_config = args.config.resolve().parents[2]/'control/config/ideal.yaml'
    local_config = args.config.resolve().parents[2]/'local_planning/config/local_cfg.yaml'
    if not control_config.exists() or not local_config.exists():
        from ament_index_python.packages import get_package_share_directory
        control_config = Path(get_package_share_directory('control'))/'config/ideal.yaml'
        local_config = Path(get_package_share_directory('local_planning'))/'config/local_cfg.yaml'
    if (yaml.safe_load(control_config.read_text())['reference_height'] != .44 or
            yaml.safe_load(local_config.read_text())['point_clearance'] != .05):
        raise ValueError('Scene conversion requires unchanged .44m reference height/.05m point clearance')
    settings = {**config['pct']['map'], **config['constraints'],
                'cost_threshold': config['pct']['search']['cost_threshold']}
    if settings['resolution'] != r or settings['slope_max'] != .4:
        raise ValueError('Scene geometry requires unchanged .05m resolution/.4rad slope')
    geometry = {'voxel_resolution': r, 'voxel_origin': [0.,0.,0.], 'all_source_points': True,
                'horizontal_pca_radius': .35, 'minimum_neighbors': 6, 'eigenvalue_ratio': .1,
                'minimum_second_eigenvalue': .0001, 'normal_z_min': .95,
                'observed_lower_surface_z_max': -.5, 'triangle_max_edge': .3,
                'triangle_slope_max': .4, 'triangle_sample_spacing': r/2,
                'unknown_policy': 'reject; no filling or extrapolation'}
    processing = args.config.resolve().parents[1]/'pct/map_processing'
    if (processing/'prepare_map.py').exists():
        sys.path.insert(0,str(processing))
        from prepare_map import observed_surface_map
    else:
        from ament_index_python.packages import get_package_prefix
        processing = Path(get_package_prefix('global_planning'))/'lib/global_planning'
        sys.path.insert(0,str(processing))
        observed_surface_map = SourceFileLoader('pct_scene_preparation',str(processing/'pct_prepare_map')).load_module().observed_surface_map
    algorithm_files = [processing/'tomogram.py',processing/'kernels.py']
    algorithm_files.append(processing/'prepare_map.py' if (processing/'prepare_map.py').exists() else processing/'pct_prepare_map')
    fingerprint = hashlib.sha256((digest(source)+json.dumps(geometry,sort_keys=True)+
        json.dumps(settings,sort_keys=True)+digest(__file__)+''.join(digest(p) for p in algorithm_files)).encode()).hexdigest()
    vertices, triangulation, accepted = surfaces(points)
    faces = triangulation.simplices[accepted]
    if not len(faces):
        raise ValueError('No observed lower surface')
    indices = np.unique(np.floor(points/r).astype(np.int32),axis=0)
    minimum = indices.min(0)*r; maximum = (indices.max(0)+1)*r
    lower = np.floor(minimum[:2]/r).astype(int)-2
    dims = np.ceil(maximum[:2]/r).astype(int)+2-lower
    origin_xy = lower*r
    center = origin_xy+(dims//2+.5)*r
    xy = np.stack(np.meshgrid(origin_xy[0]+(np.arange(dims[0])+.5)*r,
                             origin_xy[1]+(np.arange(dims[1])+.5)*r,indexing='ij'),-1).reshape(-1,2)
    simplex = triangulation.find_simplex(xy)
    support = (simplex>=0)&accepted[np.maximum(simplex,0)]
    transform = triangulation.transform[np.maximum(simplex,0)]
    bary = np.einsum('ijk,ik->ij',transform[:,:2],xy-transform[:,2])
    bary = np.column_stack([bary,1-bary.sum(1)])
    ground = np.sum(vertices[triangulation.simplices[np.maximum(simplex,0)],2]*bary,axis=1)
    # At cell centres each closed source voxel has an unambiguous XY column.
    ij = np.floor(xy/r).astype(int)
    column = {}
    for i,j,k in indices:
        column.setdefault((int(i),int(j)),[]).append((int(k)+1)*r)
    for n in np.flatnonzero(support):
        g = ground[n]
        tops = [z for z in column.get(tuple(ij[n]),[]) if g-.44<=z<=g+.39]
        if tops: ground[n] = max(g,max(tops))
    ground[~support] = np.nan
    ground = ground.reshape(tuple(dims)); support = support.reshape(tuple(dims))
    triangle_samples = sample_triangles(vertices,faces,r/2)
    box_vertices, box_faces = voxel_mesh(indices,r)
    # All six voxel surfaces, including vertical obstacle faces, share DAE/PCT geometry.
    face_samples = []
    uv = np.stack(np.meshgrid(np.linspace(0,r,3),np.linspace(0,r,3),indexing='ij'),-1).reshape(-1,2)
    base = indices*r
    for axis in range(3):
        others = [j for j in range(3) if j!=axis]
        for sign in [0.,r]:
            p = np.repeat(base[:,None,:],len(uv),axis=1)
            p[:,:,axis] += sign
            p[:,:,others[0]] += uv[:,0]; p[:,:,others[1]] += uv[:,1]
            face_samples.append(p.reshape(-1,3).astype(np.float32))
    samples = np.concatenate([triangle_samples,points]+face_samples)
    data,timing,envelope = observed_surface_map(samples,center,dims,float(minimum[2]),settings,support,ground)
    with (out/'map.pctmap').open('wb') as f:
        f.write(struct.pack('<8sIII6d',b'PCTMAP02',*data.shape[1:],r,*center,
                            settings['max_step'],settings['clearance'],envelope))
        f.write(data.tobytes())
    del samples,face_samples
    zmin = math.floor((np.nanmin(ground)-.1)/r)*r
    nz = int(math.ceil((np.nanmax(ground)+.44+.2-zmin)/r))+2
    origin = np.r_[origin_xy,zmin]
    shape = (*dims,nz)
    if int(np.prod(shape))>50000000: raise ValueError('NPZ exceeds 50 million cells')
    occupancy = np.zeros(shape,dtype=bool)
    local = indices-np.r_[lower,int(round(zmin/r))]
    inside = (local>=0).all(1)&(local<np.array(shape)).all(1)
    occupancy[tuple(local[inside].T)] = True
    local = np.floor((triangle_samples-origin)/r).astype(int)
    inside = (local>=0).all(1)&(local<np.array(shape)).all(1)
    occupancy[tuple(local[inside].T)] = True
    occupancy[[0,-1],:,:] = True
    occupancy[:,[0,-1],:] = True
    occupancy[:,:,[0,-1]] = True
    margin = math.sqrt(3)*r+r/2
    distance = distance_transform_edt(~occupancy).astype(np.float32)*r
    distance -= distance_transform_edt(occupancy).astype(np.float32)*r
    distance -= margin
    # Unknown support is a vertical column boundary: its discretisation and
    # bilinear interpolation errors each use an XY half-diagonal, without the
    # unrelated physical triangle sampling or Z errors.
    unknown_margin = math.sqrt(2)*r
    unknown_distance = (distance_transform_edt(support)-distance_transform_edt(~support))*r-unknown_margin
    distance = np.minimum(distance,unknown_distance[:,:,None]).astype(np.float32)
    occupancy[~support,:] = True
    text = lambda s: np.frombuffer(s.encode('ascii'),dtype=np.uint8)
    np.savez_compressed(out/'map.npz',version=np.int32(1),resolution=np.float64(r),
        origin=origin.astype(np.float64),axis_order=text('xyz'),occupancy=occupancy.astype(np.uint8),
        distance=distance,ground=ground[None].astype(np.float32),valid=support[None].astype(np.uint8),
        layer_z=np.array([np.nanmedian(ground)]),source_sha256=text(digest(source)),
        geometry_fingerprint=text(fingerprint),conservative_margin=np.float64(margin))
    category, body_clearance, pct_passable = display_classification(
        data, ground, support, distance, origin, r, settings)
    overlays = display_meshes(vertices, faces, ground, support, category, origin, r)
    legend_origin = [minimum[0], minimum[1]-.7, float(np.nanmedian(ground))+.02]
    legend_vertices, legend_faces = legend_mesh(legend_origin)
    overlays['display_legend'] = (legend_vertices, legend_faces, DISPLAY_COLORS['legend'])
    write_dae(out/'map.dae',np.concatenate([vertices,box_vertices]),
              np.concatenate([faces,box_faces+len(vertices)]),overlays)
    display_minimum = np.minimum(minimum,legend_vertices.min(0))
    display_maximum = np.maximum(maximum,legend_vertices.max(0))
    write_world(out/'map.world',display_minimum,display_maximum,list(overlays))
    metadata = {'map_kind':'pcd_surface_voxel_union_v1','terrain_model':'observed_surface',
        'fingerprint':fingerprint,'source_sha256':digest(source),'source':'source/clean_map.pcd',
        'parameters':settings,'geometry':geometry,'shape':list(data.shape),'center':center.tolist(),
        'origin':origin.tolist(),'resolution':r,'regions':[],'point_count':len(points),
        'triangle_count':len(faces),'voxel_count':len(indices),'gpu_timing_ms':timing,
        'coordinates':{'frame':'world','unit':'m','transform':'identity; source coordinates retained',
                       'evidence':'near-horizontal observed surfaces around z=-1 and z=1.9; near-vertical walls; no registration'},
        'distance_field':{'axis_order':'xyz','origin':'lower corner','samples':'cell centers',
                          'conservative_margin':margin,'outside':'reject','unknown':'occupied',
                          'composition':'minimum of physical signed EDT and vertical unknown-support signed EDT',
                          'unknown_conservative_margin':unknown_margin,
                          'physical_boundary':'all six volume faces occupied'},
        'display': {'reference_height': .44, 'point_clearance': .05,
                    'classification': 'observed support AND compatible PCT ground/cost/ceiling AND SCAN clearance at ground+reference_height',
                    'cell_counts': {'unknown':int((category==0).sum()),
                                    'blocked':int((category==1).sum()),
                                    'passable':int((category==2).sum())},
                    'unknown': 'no color; no fabricated floor',
                    'collision_submesh': 'original', 'overlay_model': 'display_overlay',
                    'bounds': {'min':display_minimum.tolist(),'max':display_maximum.tolist()},
                    'material': 'single emissive category color; ambient/diffuse zero',
                    'surface': 'observed triangles clipped to classified grid cells; display-only offset .008m'},
        'generation_seconds':time.monotonic()-started}
    (out/'map.json').write_text(json.dumps(metadata,indent=2)+'\n')
    # Select nearby unambiguous shared-grid centres, preserving the investigated route.
    presets = {}
    for name,target in [('PCD-START',[-15.45,-.5]),('PCD-GOAL',[-13.8,-1.1]),
                        ('PCD-LONG-START',[-11.775,-.175]),('PCD-LONG-GOAL',[-13.575,-1.525])]:
        order = np.argsort(np.linalg.norm(xy-np.array(target),axis=1))
        chosen = None
        for flat in order[:100]:
            i,j = np.unravel_index(flat,tuple(dims))
            if support[i,j] and pct_passable[i,j]:
                zindex = (ground[i,j]+.44-zmin)/r-.5
                k = int(np.floor(zindex)); fraction = zindex-k
                value = ((1-fraction)*distance[i,j,k]+fraction*distance[i,j,k+1]
                         if 0<=k<nz-1 else -math.inf)
                if value>.10:
                    chosen=(i,j);break
        if chosen is None: raise ValueError('No safe preset near '+name)
        i,j=chosen
        presets[name]={'label':name,'pose':[float(xy[np.ravel_multi_index(chosen,tuple(dims)),0]),
            float(xy[np.ravel_multi_index(chosen,tuple(dims)),1]),float(ground[i,j]),0.]}
    (out/'source').mkdir(exist_ok=True)
    target = out/'source/clean_map.pcd'
    if source.resolve()!=target.resolve(): target.write_bytes(source.read_bytes())
    files={'pcd':'source/clean_map.pcd','pctmap':'map.pctmap','metadata':'map.json',
           'npz':'map.npz','mesh':'map.dae','world':'map.world'}
    manifest={'version':1,'id':'wisal-pcd','name':'Wisal PCD · 实测环境','frame':'world',
        'files':files,'sha256':{k:digest(out/v) for k,v in files.items()},
        'source_sha256':digest(source),'geometry_fingerprint':fingerprint,
        'coordinates':metadata['coordinates'],'features':{'dynamic_obstacles':False},
        'points':presets,'start':'PCD-LONG-START','goal':'PCD-LONG-GOAL'}
    (out/'map.yaml').write_text(yaml.safe_dump(manifest,allow_unicode=True,sort_keys=False))
    print(json.dumps({'output':str(out),'points':presets,'seconds':time.monotonic()-started},indent=2),flush=True)


if __name__=='__main__':
    main()
