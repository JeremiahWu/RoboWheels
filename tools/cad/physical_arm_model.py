#!/usr/bin/env python3
"""Generate an eight-joint RViz shadow model without mock control hardware."""
import math
import xml.etree.ElementTree as ET


def model(source):
    root = ET.fromstring(source)
    for element in list(root.findall('ros2_control')):
        root.remove(element)
    for side in ('right', 'left'):
        joint = root.find(f"joint[@name='{side}_wrist_roll']")
        joint.set('type', 'fixed')
        for tag in ('axis', 'limit'):
            for child in list(joint.findall(tag)):
                joint.remove(child)
        # CAD assembly has arms extended. Rotate shoulder-pitch reference by
        # +90 degrees about its existing axis to approximate arms lowered.
        # This is a visual reference, not a measured camera/robot calibration.
        shoulder = root.find(f"joint[@name='{side}_shoulder_pitch']")
        x, y, z = map(float, shoulder.find('axis').get('xyz').split())
        norm = math.sqrt(x*x+y*y+z*z)
        x, y, z = x/norm, y/norm, z/norm
        a = math.pi/2
        c, s, t = math.cos(a), math.sin(a), 1-math.cos(a)
        r00, r10, r20 = c+x*x*t, y*x*t+z*s, z*x*t-y*s
        r21, r22 = z*y*t+x*s, c+z*z*t
        roll, pitch, yaw = math.atan2(r21, r22), math.atan2(-r20, math.hypot(r00, r10)), math.atan2(r10, r00)
        shoulder.find('origin').set('rpy', f'{roll} {pitch} {yaw}')
    return ET.tostring(root, encoding='unicode')


if __name__ == '__main__':
    import sys
    from pathlib import Path
    Path(sys.argv[2]).write_text(model(Path(sys.argv[1]).read_text()))
