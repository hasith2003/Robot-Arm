"""Generate a separate scene with a different ground-level pickup position."""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def write_world(x, y, destination):
    if not all(math.isfinite(value) for value in (x, y)):
        raise ValueError('Coordinates must be finite')
    # Box A's footprint plus the small cube half-width.
    if abs(x - .5) <= .175 and abs(y - .3) <= .175:
        raise ValueError('Ground-level cube would overlap Box A')
    source = Path(__file__).resolve().parent / 'src/gp7_simulation/worlds/pick_place.sdf'
    tree = ET.parse(source)
    tree.find("./world/model[@name='box_b']/pose").text = f'{x} {y} 0.025 0 0 0'
    tree.write(destination, encoding='unicode', xml_declaration=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('output')
    args = parser.parse_args()
    write_world(args.x, args.y, args.output)
