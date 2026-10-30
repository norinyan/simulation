"""Read-only conversion of the actual planning cache and Gazebo collision World."""
import math
import hashlib
import json
import struct
import zipfile
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import yaml


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


def world_map(filename, scene=None):
    if scene is not None and scene['id'] == 'wisal-pcd':
        return {'file': str(filename), 'boxes': [], 'bounds': scene.get('display_bounds',scene['bounds']),
                'mesh_url': '/api/map/mesh?map_id=wisal-pcd', 'map_id': scene['id'],
                'display_overlay': bool(scene.get('display_overlay'))}
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


def _relative_file(directory, value):
    if not isinstance(value, str) or not value or Path(value).is_absolute():
        raise ValueError('Map paths must be nonempty relative paths')
    candidate = (directory/value).resolve()
    if not candidate.is_relative_to(directory.resolve()):
        raise ValueError('Map resource escapes its directory: '+value)
    if not candidate.is_file():
        raise ValueError('Map resource missing: '+value)
    return candidate


def _sha(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def _validate_completion(manifest, metadata):
    terrain = metadata.get('terrain_model')
    if terrain == 'observed_surface':
        if any(record.get('simulation_only') or record.get('completion') for record in [manifest,metadata]):
            raise ValueError('Observed map cannot declare simulation completion')
        return
    if terrain != 'observed_surface_with_simulation_completion':
        raise ValueError('Unsupported map terrain model')
    if manifest.get('simulation_only') is not True or metadata.get('simulation_only') is not True:
        raise ValueError('Completed map must be explicitly simulation-only')
    completion = metadata.get('completion')
    if not isinstance(completion,dict) or not isinstance(completion.get('spec'),dict) or not completion['spec']:
        raise ValueError('Missing simulation completion specification')
    canonical = lambda value: json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)
    if canonical(completion) != canonical(manifest.get('completion')):
        raise ValueError('Simulation completion manifest/metadata mismatch')
    if hashlib.sha256(canonical(completion['spec']).encode()).hexdigest() != completion.get('spec_sha256'):
        raise ValueError('Simulation completion specification fingerprint mismatch')
    bounds = completion.get('bounds')
    if not isinstance(bounds,dict) or set(bounds) != {'min','max'}:
        raise ValueError('Missing simulation completion bounds')
    lower,upper = np.asarray(bounds['min'],dtype=float),np.asarray(bounds['max'],dtype=float)
    if (lower.shape!=(3,) or upper.shape!=(3,) or not np.isfinite([lower,upper]).all() or
            np.any(lower>upper) or np.any(lower[:2]>=upper[:2])):
        raise ValueError('Invalid simulation completion bounds')


def validate_manifest(manifest_path):
    """Validate every resource before authorizing this observed map for a task."""
    path = Path(manifest_path).resolve()
    try:
        manifest = yaml.safe_load(path.read_text())
        if not isinstance(manifest, dict) or manifest.get('version') != 1 or manifest.get('id') != 'wisal-pcd':
            raise ValueError('Unsupported map manifest version or id')
        if manifest.get('frame') != 'world' or not isinstance(manifest.get('name'), str):
            raise ValueError('Invalid map frame or name')
        keys = {'pcd','pctmap','metadata','npz','mesh','world'}
        if set(manifest.get('files',{})) != keys or set(manifest.get('sha256',{})) != keys:
            raise ValueError('Map manifest must identify all six resources')
        files = {key:_relative_file(path.parent,value) for key,value in manifest['files'].items()}
        for key, resource in files.items():
            if _sha(resource) != manifest['sha256'][key]:
                raise ValueError('Map resource SHA-256 mismatch: '+key)
        source_hash, geometry = manifest['source_sha256'], manifest['geometry_fingerprint']
        for value in [source_hash,geometry]:
            if not isinstance(value,str) or len(value)!=64 or any(c not in '0123456789abcdef' for c in value):
                raise ValueError('Invalid map fingerprint')
        if manifest['sha256']['pcd'] != source_hash:
            raise ValueError('Map source fingerprint mismatch')
        metadata = json.loads(files['metadata'].read_text())
        if (metadata.get('map_kind') != 'pcd_surface_voxel_union_v1' or
                metadata.get('source_sha256') != source_hash or metadata.get('fingerprint') != geometry or
                metadata.get('regions') != []):
            raise ValueError('Map metadata identity mismatch')
        _validate_completion(manifest,metadata)
        if _relative_file(path.parent,metadata.get('source')) != files['pcd']:
            raise ValueError('Map metadata source path mismatch')
        header = struct.Struct('<8sIII6d')
        with files['pctmap'].open('rb') as stream:
            raw_header = stream.read(header.size)
        magic,layers,nx,ny,res,cx,cy,max_step,clearance,inflation = header.unpack(raw_header)
        if (magic!=b'PCTMAP02' or min(layers,nx,ny)<1 or layers*nx*ny>50000000 or
                files['pctmap'].stat().st_size != header.size+5*layers*nx*ny*4 or
                metadata.get('shape') != [5,layers,nx,ny] or
                not np.isfinite([res,cx,cy,max_step,clearance,inflation]).all() or min(res,max_step,clearance)<=0 or
                not np.allclose(metadata.get('center'),[cx,cy],rtol=0,atol=1e-9) or
                metadata.get('resolution')!=res):
            raise ValueError('Map PCT shape or coordinate mismatch')
        parameters = metadata['parameters']
        if (parameters['resolution']!=res or parameters['max_step']!=max_step or
                parameters['clearance']!=clearance):
            raise ValueError('Map PCT parameter mismatch')
        # Reject oversized zip payloads before NumPy allocates decompressed arrays.
        with zipfile.ZipFile(files['npz']) as archive:
            if sum(item.file_size for item in archive.infolist()) > 400000000:
                raise ValueError('Map NPZ exceeds decompression budget')
        with np.load(files['npz'],allow_pickle=False) as arrays:
            required = {'version','resolution','origin','axis_order','occupancy','distance','ground','valid',
                        'layer_z','source_sha256','geometry_fingerprint','conservative_margin'}
            if set(arrays.files)!=required:
                raise ValueError('Map NPZ schema mismatch')
            ascii_value = lambda key: arrays[key].tobytes().decode('ascii')
            if (arrays['version'].shape!=() or arrays['version'].dtype!=np.dtype('int32') or int(arrays['version'])!=1 or
                    arrays['resolution'].shape!=() or arrays['resolution'].dtype!=np.dtype('float64') or float(arrays['resolution'])!=res or
                    arrays['origin'].shape!=(3,) or arrays['origin'].dtype!=np.dtype('float64') or
                    not np.isfinite(arrays['origin']).all() or ascii_value('axis_order')!='xyz' or
                    ascii_value('source_sha256')!=source_hash or ascii_value('geometry_fingerprint')!=geometry):
                raise ValueError('Map NPZ version, coordinate or fingerprint mismatch')
            for key,size in [('axis_order',3),('source_sha256',64),('geometry_fingerprint',64)]:
                if arrays[key].dtype!=np.dtype('uint8') or arrays[key].shape!=(size,):
                    raise ValueError('Map NPZ string encoding mismatch: '+key)
            origin = arrays['origin']
            if not np.allclose(origin,metadata.get('origin'),rtol=0,atol=1e-9):
                raise ValueError('Map NPZ/PCT origin mismatch')
            if not np.allclose(origin[:2]+(np.array([nx,ny])//2+.5)*res,[cx,cy],rtol=0,atol=1e-8):
                raise ValueError('Map NPZ/PCT grid mismatch')
            occupancy,distance = arrays['occupancy'],arrays['distance']
            ground,valid = arrays['ground'],arrays['valid']
            if (occupancy.dtype!=np.dtype('uint8') or distance.dtype!=np.dtype('float32') or
                    occupancy.ndim!=3 or occupancy.shape!=distance.shape or occupancy.shape[:2]!=(nx,ny) or
                    min(occupancy.shape)<2 or occupancy.size>50000000 or
                    ground.dtype!=np.dtype('float32') or valid.dtype!=np.dtype('uint8') or
                    ground.ndim!=3 or ground.shape!=valid.shape or ground.shape[1:]!=(nx,ny) or
                    arrays['layer_z'].shape!=(ground.shape[0],) or arrays['layer_z'].dtype!=np.dtype('float64') or
                    not np.isfinite(arrays['layer_z']).all()):
                raise ValueError('Map NPZ array shape or dtype mismatch')
            if (not np.isfinite(distance).all() or np.any(occupancy>1) or np.any(valid>1) or
                    np.any(~np.isfinite(ground[valid.astype(bool)])) or
                    np.any(~np.isnan(ground[~valid.astype(bool)]))):
                raise ValueError('Map NPZ contains invalid numerical values')
            unknown = ~np.any(valid.astype(bool),axis=0)
            if np.any(occupancy[unknown]!=1) or np.any(distance[unknown]>0):
                raise ValueError('Map NPZ exposes unknown space as free')
            margin = float(arrays['conservative_margin'])
            if (arrays['conservative_margin'].shape!=() or arrays['conservative_margin'].dtype!=np.dtype('float64') or not math.isfinite(margin) or
                    abs(margin-(math.sqrt(3)*res+res/2))>1e-9):
                raise ValueError('Map NPZ conservative distance margin mismatch')
            points = manifest.get('points')
            if not isinstance(points,dict) or len(points)<2:
                raise ValueError('Map must define at least two preset points')
            pct = np.memmap(files['pctmap'],dtype='<f4',mode='r',offset=header.size,shape=(5,layers,nx,ny))
            for value in points.values():
                pose_value = np.asarray(value['pose'],dtype=float)
                if pose_value.shape!=(4,) or not np.isfinite(pose_value).all():
                    raise ValueError('Invalid map preset pose')
                ij=np.floor((pose_value[:2]-origin[:2])/res).astype(int)
                if (np.any(ij<0) or np.any(ij>=[nx,ny]) or not np.any(valid[:,ij[0],ij[1]]) or
                        not np.any(np.isfinite(ground[:,ij[0],ij[1]])&
                                   (abs(ground[:,ij[0],ij[1]]-pose_value[2])<=1e-6))):
                    raise ValueError('Map preset has no matching observed support')
                pct_ground,pct_cost=pct[3,:,ij[0],ij[1]],pct[0,:,ij[0],ij[1]]
                pct_ceiling=pct[4,:,ij[0],ij[1]]
                if not np.any(np.isfinite(pct_ground)&np.isfinite(pct_cost)&(abs(pct_ground-pose_value[2])<=.01)&
                              (pct_cost<=parameters['cost_threshold'])&
                              (~np.isfinite(pct_ceiling)|(pct_ceiling-pct_ground>=clearance-1e-6))):
                    raise ValueError('Map preset has no compatible PCT support')
                coordinate=(np.r_[pose_value[:2],pose_value[2]+.44]-origin)/res-.5
                index=np.floor(coordinate).astype(int);fraction=coordinate-index
                if np.any(index<0) or np.any(index+1>=distance.shape):
                    raise ValueError('Map preset body outside distance field')
                body_clearance=0.
                for dx in [0,1]:
                    for dy in [0,1]:
                        for dz in [0,1]:
                            offset=np.array([dx,dy,dz]);weight=np.prod(np.where(offset,fraction,1-fraction))
                            body_clearance+=weight*distance[tuple(index+offset)]
                if body_clearance<.05:
                    raise ValueError('Map preset body violates conservative clearance')
        if manifest.get('start') not in points or manifest.get('goal') not in points or manifest['start']==manifest['goal']:
            raise ValueError('Invalid map preset defaults')
        if manifest.get('features')!={'dynamic_obstacles':False}:
            raise ValueError('Observed map must disable dynamic obstacles')
        world = ET.parse(files['world']).getroot().find('world')
        if world is None:
            raise ValueError('Map World has no world element')
        uris = world.findall('.//mesh/uri')
        if not uris or any(_relative_file(files['world'].parent,uri.text)!=files['mesh'] for uri in uris):
            raise ValueError('Map World/DAE reference mismatch')
        dae = ET.parse(files['mesh']).getroot()
        if dae.tag!='{http://www.collada.org/2005/11/COLLADASchema}COLLADA':
            raise ValueError('Map mesh is not a readable COLLADA asset')
        display = metadata.get('display')
        if display:
            namespace = {'c':'http://www.collada.org/2005/11/COLLADASchema'}
            names = [node.get('name') for node in dae.findall('.//c:visual_scene/c:node',namespace)]
            expected = {'original','display_legend'}
            allowed = expected | {'display_passable','display_blocked'}
            if len(names)!=len(set(names)) or not expected <= set(names) <= allowed:
                raise ValueError('Map display submesh names mismatch')
            models = world.findall('model')
            if [model.get('name') for model in models]!=['observed_map','display_overlay']:
                raise ValueError('Map display model structure mismatch')
            base, overlay = models
            collisions = world.findall('.//collision')
            if (len(collisions)!=1 or collisions[0] not in base.findall('.//collision') or
                    collisions[0].findtext('geometry/mesh/submesh/name')!='original' or
                    overlay.findall('.//collision') or overlay.findtext('static')!='true'):
                raise ValueError('Map display geometry must never enter collision')
            base_visuals = base.findall('.//visual')
            if len(base_visuals)!=1 or base_visuals[0].findtext('geometry/mesh/submesh/name')!='original':
                raise ValueError('Map original visual must select original submesh')
            overlay_visuals = overlay.findall('.//visual')
            selected = [v.findtext('geometry/mesh/submesh/name') for v in overlay_visuals]
            if len(selected)!=len(set(selected)) or set(selected)!=set(names)-{'original'}:
                raise ValueError('Map overlay visual/submesh mismatch')
            for element in collisions+base_visuals+overlay_visuals:
                geometries = element.findall('geometry')
                if (len(geometries)!=1 or len(list(geometries[0]))!=1 or
                        list(geometries[0])[0].tag!='mesh'):
                    raise ValueError('Map display bundle only permits its declared mesh geometry')
            meshes = world.findall('.//mesh')
            if (len(meshes)!=2+len(selected) or len(uris)!=len(meshes) or
                    any(mesh.findtext('submesh/center')!='false' for mesh in meshes)):
                raise ValueError('Map meshes must retain explicit uncentered submesh selection')
            if display.get('collision_submesh')!='original' or display.get('overlay_model')!='display_overlay':
                raise ValueError('Map display metadata mismatch')
        elif len(uris)!=2:
            raise ValueError('Map World/DAE reference mismatch')
        blob = files['pcd'].read_bytes()
        pcd_header,payload = blob.split(b'DATA binary\n',1)
        fields={line.split()[0]:line.split()[1:] for line in pcd_header.decode().splitlines() if line and not line.startswith('#')}
        if (fields.get('FIELDS')!=['x','y','z','intensity'] or fields.get('SIZE')!=['4']*4 or
                fields.get('TYPE')!=['F']*4 or fields.get('COUNT')!=['1']*4 or fields.get('HEIGHT')!=['1']):
            raise ValueError('Map source PCD format mismatch')
        source_points = np.frombuffer(payload,dtype='<f4').reshape(-1,4)[:,:3]
        if (len(source_points)!=int(fields['POINTS'][0]) or len(source_points)!=metadata['point_count'] or
                not np.isfinite(source_points).all()):
            raise ValueError('Map source PCD count or coordinate mismatch')
        minimum = np.floor(source_points.astype(float).min(0)/res)*res
        maximum = (np.floor(source_points.astype(float).max(0)/res)+1)*res
        return {key:manifest[key] for key in ['id','name','points','start','goal','features','source_sha256','geometry_fingerprint']} | {
            'files':files,'manifest':path,'display_overlay':bool(display),
            'display_bounds':display['bounds'] if display else {'min':minimum.tolist(),'max':maximum.tolist()},'bounds':{'min':minimum.tolist(),'max':maximum.tolist()}}
    except (OSError,KeyError,TypeError,UnicodeError,struct.error,zipfile.BadZipFile,ET.ParseError) as error:
        raise ValueError('Invalid map manifest/resources: '+str(error)) from error


def _new_manifest(old_global_config):
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    except ImportError:
        return Path(old_global_config).resolve().parents[2]/'sim_bringup/maps/map.yaml'
    try:
        return Path(get_package_share_directory('sim_bringup'))/'maps/map.yaml'
    except PackageNotFoundError:
        return Path(old_global_config).resolve().parents[2]/'sim_bringup/maps/map.yaml'


def load_scene(map_id, old_global_config):
    if map_id=='wisal-pcd':
        return validate_manifest(_new_manifest(old_global_config))
    if map_id!='three-floor':
        raise ValueError('Unknown map id: '+str(map_id))
    config_path = Path(old_global_config).resolve()
    config = yaml.safe_load(config_path.read_text())
    resolve = lambda value: (config_path.parent/Path(value)).resolve()
    return {'id':'three-floor','name':'原三层楼','points':config['points'],
            'start':config['planning']['start'],'goal':config['planning']['goal'],
            'features':{'dynamic_obstacles':True},'files':{'world':resolve(config['pct']['world_file']),
            'pctmap':resolve(config['planning']['map_file'])},'manifest':None,
            'source_sha256':None,'geometry_fingerprint':None}


def scene_catalog(old_global_config):
    result = []
    for map_id,name in [('three-floor','原三层楼'),('wisal-pcd','Wisal PCD · 实测环境')]:
        try:
            scene = load_scene(map_id,old_global_config)
            entry = {key:scene[key] for key in ['id','name','points','start','goal','features']}
            entry.update(available=True,reason='')
        except ValueError as error:
            # A broken optional map stays visible with its error, while the
            # original scene remains usable. Selecting it still fails strictly.
            entry = {'id':map_id,'name':name,'points':{},'start':None,'goal':None,
                     'features':{'dynamic_obstacles':map_id=='three-floor'},
                     'available':False,'reason':str(error)}
        result.append(entry)
    return result
