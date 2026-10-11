"""Generate original project gripper STL meshes in metres (no CAD dependencies).

tool0 is the flange face; its +Z axis points out along the gripper.
The mount spans Z=0..8 mm and the body spans Z=8..65 mm.
Each jaw is a separate rigid mesh, local Z=0..65 mm, local Y=+-7.5 mm.
These are custom simulation parts, not manufacturer CAD.
"""
from pathlib import Path
import math
import struct


def prism(profile, low, high, axis='z'):
    """Closed convex profile extrusion; profile must be counterclockwise."""
    def vertex(p, height):
        # (z,x) cyclic permutation preserves orientation for the jaw profile.
        return (p[1], height, p[0]) if axis == 'y' else (p[0], p[1], height)
    bottom = [vertex(p, low) for p in profile]
    top = [vertex(p, high) for p in profile]
    triangles = []
    for i in range(1, len(profile) - 1):
        triangles.extend([(bottom[0], bottom[i + 1], bottom[i]),
                          (top[0], top[i], top[i + 1])])
    for i in range(len(profile)):
        j = (i + 1) % len(profile)
        triangles.extend([(bottom[i], bottom[j], top[j]),
                          (bottom[i], top[j], top[i])])
    return triangles


def write_stl(path, triangles):
    with path.open('wb') as stream:
        stream.write(b'EN3563 custom parallel gripper, metres'.ljust(80, b' '))
        stream.write(struct.pack('<I', len(triangles)))
        for a, b, c in triangles:
            u, v = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
            n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
            length = math.sqrt(sum(x*x for x in n))
            n = [x/length for x in n]
            stream.write(struct.pack('<12fH', *n, *a, *b, *c, 0))


def main():
    folder = Path(__file__).resolve().parent
    mount = [(0.030*math.cos(i*math.tau/48), 0.030*math.sin(i*math.tau/48)) for i in range(48)]
    body = [(-.029, -.050), (.029, -.050), (.035, -.044), (.035, .044),
            (.029, .050), (-.029, .050), (-.035, .044), (-.035, -.044)]
    # Jaw profile is (z, x), with rounded-off tip approximated by chamfers.
    jaw = [(0, -.018), (.059, -.018), (.065, -.012), (.065, .012), (.059, .018), (0, .018)]
    for filename, triangles in [
        ('flange_mount.stl', prism(mount, 0, .008)),
        ('body.stl', prism(body, .008, .065)),
        ('jaw.stl', prism(jaw, -.0075, .0075, axis='y')),
    ]:
        write_stl(folder / filename, triangles)
        print(filename, len(triangles), 'triangles')


if __name__ == '__main__':
    main()
