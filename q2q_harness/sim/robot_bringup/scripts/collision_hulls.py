"""Optional conservative convex finger collisions, original visual/joints retained.

Detailed CAD meshes have 13k--17k triangles each. Store hulls and an audit in the
generated setup; never overwrite upstream assets. Hulls fill concavities, so this
is an explicit simulation approximation, not identical collision geometry.
"""
import json
from pathlib import Path
import struct
import numpy as np
from scipy.spatial import ConvexHull
from ament_index_python.packages import get_package_share_directory


def apply(root, setup):
    destination = Path(setup)/'collision_hulls'
    destination.mkdir(exist_ok=True)
    audit = []
    for link in root.findall('link'):
        if 'finger_' not in link.get('name', ''):
            continue
        for mesh in link.findall('collision/geometry/mesh'):
            uri = mesh.get('filename')
            if not uri.startswith('package://'):
                raise ValueError('Expected original package mesh, regenerate first: '+uri)
            package, relative = uri[len('package://'):].split('/', 1)
            data = (Path(get_package_share_directory(package))/relative).read_bytes()
            count = struct.unpack_from('<I', data, 80)[0]
            if len(data) != 84+50*count:
                raise ValueError('Expected binary STL: '+uri)
            vertices = np.unique(np.array([struct.unpack_from('<9f', data, 84+50*i+12)
                for i in range(count)]).reshape(-1, 3), axis=0)
            hull = ConvexHull(vertices)
            output = bytearray(b'Q2Q conservative collision hull'.ljust(80, b'\0'))
            output.extend(struct.pack('<I', len(hull.simplices)))
            for face, equation in zip(hull.simplices, hull.equations):
                points = vertices[face].copy()
                normal = np.cross(points[1]-points[0], points[2]-points[0])
                if np.dot(normal, equation[:3]) < 0:
                    points[[1, 2]] = points[[2, 1]]
                output.extend(struct.pack('<12fH', *equation[:3], *points.reshape(-1), 0))
            target = destination/(link.get('name')+'.stl')
            target.write_bytes(output)
            mesh.set('filename', 'file://'+str(target))
            item = dict(link=link.get('name'), original=uri, triangles_before=count,
                triangles_after=len(hull.simplices), bounds=[vertices.min(0).tolist(), vertices.max(0).tolist()],
                approximation='convex hull fills CAD concavities; visuals and moving joints unchanged')
            audit.append(item)
            print('[collision hull]', item['link'], count, '->', len(hull.simplices), flush=True)
    if len(audit) != 4:
        raise ValueError('Expected four finger collision meshes')
    (destination/'audit.json').write_text(json.dumps(audit, indent=2), encoding='utf-8')
