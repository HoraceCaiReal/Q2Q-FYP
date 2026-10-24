#!/usr/bin/env python3
"""Generate the explicit ground-truth 2D map; this is not a SLAM result.

Supports the current world's unrotated models and box collisions at zero local
offset. Fail on unsupported offsets/rotations rather than silently making a map.
All box heights are projected into the 2D navigation plane, including low bins.
"""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def boxes_of(source):
    boxes = []
    for model in ET.parse(source).getroot().find('world').findall('model'):
        pose = list(map(float, model.findtext('pose', '0 0 0 0 0 0').split()))
        if any(abs(v) > 1e-9 for v in pose[3:]):
            raise ValueError('Rotated models are not supported: ' + model.get('name'))
        for link in model.findall('link'):
            if link.find('pose') is not None:
                raise ValueError('Link offsets require an extended map generator')
            for collision in link.findall('collision'):
                if collision.find('pose') is not None:
                    raise ValueError('Collision offsets require an extended map generator')
                size = collision.findtext('geometry/box/size')
                if size is None:
                    if collision.find('geometry/plane') is not None:
                        continue
                    raise ValueError('Only box collisions and the ground plane are supported')
                x, y, _ = map(float, size.split())
                boxes.append((pose[0] - x/2, pose[0] + x/2, pose[1] - y/2, pose[1] + y/2))
    return boxes


def generate(source, output):
    boxes = boxes_of(source)
    resolution, x0, y0, width, height = 0.05, -6.5, -5.5, 260, 220
    pixels = bytearray()
    for row in range(height):
        y = y0 + (height - row - 0.5) * resolution
        for col in range(width):
            x = x0 + (col + 0.5) * resolution
            occupied = any(a <= x <= b and c <= y <= d for a, b, c, d in boxes)
            pixels.append(0 if occupied else 254)
    output.parent.mkdir(parents=True, exist_ok=True)
    header = 'P5\n# Ground-truth XY projection of q2q_warehouse.sdf\n%d %d\n255\n' % (width, height)
    output.with_suffix('.pgm').write_bytes(header.encode() + pixels)
    output.with_suffix('.yaml').write_text(
        'image: %s\nmode: trinary\nresolution: 0.05\norigin: [-6.5, -5.5, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n' % output.with_suffix('.pgm').name,
        encoding='utf-8')
    print('MAP_GENERATED boxes=%s occupied=%s size=%sx%s' % (len(boxes), pixels.count(0), width, height))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    base = Path(__file__).resolve().parents[1]
    parser.add_argument('--world', type=Path, default=base / 'worlds/q2q_warehouse.sdf')
    parser.add_argument('--output', type=Path, default=base / 'maps/q2q_warehouse_ground_truth')
    args = parser.parse_args()
    generate(args.world, args.output)
