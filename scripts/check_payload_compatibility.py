#!/usr/bin/env python3
"""Read-only compatibility gate against the checkout the user actually has."""
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

root=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else Path.cwd()
checks=[]
def check(title,passed,detail=''):
    checks.append(passed)
    print(('PASS' if passed else 'FAIL')+' '+title+((' — '+detail) if detail else ''))

world=root/'simulation/worlds/tricopter_full_mission.sdf'
model=root/'simulation/models/payload/model.sdf'
compose=root/'compose-full-sim.yaml'
base=root/'compose.yaml'
try:
    w=ET.parse(world).getroot()
    check('World contains tricopter',(w.find(".//include/[uri='model://tricopter_vtol']") is not None))
    check('World contains ground runway',(w.find(".//include/[uri='model://runway']") is not None))
    check('Landing marker unchanged',w.find(".//model[@name='landing_marker']") is not None)
    check('Payload included once',len(w.findall(".//include/[uri='model://payload']"))==1)
    check('No extra ArduPilot instances',len(w.findall(".//include/[uri='model://tricopter_vtol']"))==1)
except (OSError,ET.ParseError) as e:check('World parseable',False,str(e))
try:
    m=ET.parse(model).getroot().find('model')
    check('Payload has 3 revolute joints',len(m.findall("joint[@type='revolute']"))==3)
    check('Payload is dynamic',m.findtext('static')=='false')
    check('Wheel force controllers',len(m.findall("plugin[use_force_commands='true']"))==3)
except (OSError,ET.ParseError) as e:check('Payload parseable',False,str(e))
try:
    c=compose.read_text()
    check('Full mission world selected','tricopter_full_mission.sdf' in c)
    check('Plane SITL selected','ArduPlane' in c)
    check('Camera topic untouched','AVION_IMAGE_TOPIC' in c)
    check('Existing model resource path', 'simulation/models' in (root/'Dockerfile').read_text())
except OSError as e:check('Compose/Docker readable',False,str(e))
check('Standalone payload physics test', (root/'simulation/worlds/payload_physics_test.sdf').is_file())
print(f'\nStatic gates: {sum(checks)}/{len(checks)} passed. Real Gazebo/DART behavior NOT checked by this script.')
if not all(checks):sys.exit(1)
