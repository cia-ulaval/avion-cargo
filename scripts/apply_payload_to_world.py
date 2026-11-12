#!/usr/bin/env python3
"""Idempotent, minimal text-only edit to the existing aviation mission world.

Preserves all upstream ArduPilot / camera / marker elements and formatting.
Refuses a repeated or suspicious inclusion rather than overwriting the world.
"""
import argparse
from pathlib import Path
from xml.etree import ElementTree as ET

INCLUDE = '''    <!-- Ground payload demo: outside the 1.0185 m landing marker, inside the 2.4384 m DLZ. -->
    <include>
      <uri>model://payload</uri>
      <name>payload_demo</name>
      <pose>0.80 0.80 0 0 0 0</pose>
    </include>
'''


def install(world):
    old = world.read_text()
    root = ET.fromstring(old)
    if root.tag != 'sdf' or root.find('./world') is None:
        raise RuntimeError('Not a Gazebo SDF world')
    if 'model://payload' in old or '<name>payload_demo</name>' in old:
        print('Already integrated; unchanged:', world)
        return False
    if 'model://tricopter_vtol' not in old or 'landing_marker' not in old:
        raise RuntimeError('Expected tricopter mission world not found; not modifying file')
    tail = '  </world>\n</sdf>'
    if old.count(tail) != 1:
        raise RuntimeError('Unexpected world ending; not modifying file')
    new = old.replace(tail, INCLUDE + tail)
    ET.fromstring(new)
    world.write_text(new)
    print('Added standalone payload to',world)
    return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('world', nargs='?', type=Path, default=Path('simulation/worlds/tricopter_full_mission.sdf'))
    install(parser.parse_args().world)
