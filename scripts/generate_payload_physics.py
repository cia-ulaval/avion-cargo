#!/usr/bin/env python3
"""Generate a provisional 3-omniwheel rigid-body Gazebo Sim model.

DO NOT interpret simulated slip as experimentally validated omniwheel physics.
The wheel-contact implementation is isolated for future detailed roller wheels.
"""
import argparse
import math
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'simulation/models/payload/model.sdf'


def fmt(v):
    return f'{v:.9g}'


def inertia_cylinder(m, radius, length, axis='z'):
    longi = .5*m*radius**2
    trans = m*(3*radius**2 + length**2)/12
    return (longi,trans,trans) if axis=='x' else ((trans,longi,trans) if axis=='y' else (trans,trans,longi))


def inertial(m, diagonal, pose=None):
    offset = f'<pose>{pose}</pose>' if pose is not None else ''
    return f'<inertial>{offset}<mass>{fmt(m)}</mass><inertia><ixx>{fmt(diagonal[0])}</ixx><iyy>{fmt(diagonal[1])}</iyy><izz>{fmt(diagonal[2])}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'


def geometry(kind, size):
    if kind == 'cylinder':
        return f'<cylinder><radius>{fmt(size[0])}</radius><length>{fmt(size[1])}</length></cylinder>'
    return f'<box><size>{" ".join(map(fmt,size))}</size></box>'


def visual(name, pose, kind, size, color):
    return f'<visual name="{name}"><pose>{pose}</pose><geometry>{geometry(kind,size)}</geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>'


def wheel_section(i, angle, r, radius, width, m):
    x = r * math.cos(angle)
    y = r * math.sin(angle)
    z = radius + .001
    # Wheel cylinder's local z axis is oriented along outward radial direction.
    pose = f'{fmt(x)} {fmt(y)} {fmt(z)} 0 {fmt(math.pi/2)} {fmt(angle)}'
    name = f'wheel_{i}'
    di = inertia_cylinder(m, radius, width)
    # The high/low friction vectors are local to each wheel contact.
    # NOTE: DART's response to anisotropic mu2/fdir1 MUST be checked in Gazebo.
    collision = f'''<collision name="{name}_contact"><geometry>{geometry('cylinder',(radius,width))}</geometry>
      <surface><friction><ode><mu>1.2</mu><mu2>0.02</mu2><fdir1>0 1 0</fdir1></ode></friction></surface></collision>'''
    link = f'<link name="{name}"><pose>{pose}</pose>{inertial(m,di)}{collision}{visual(name+"_visual","0 0 0 0 0 0", "cylinder", (radius,width), "0.08 0.08 0.09 1")}</link>'
    # Joint frame fixed at wheel origin, z aligned with wheel cylinder axle.
    joint = f'''<joint name="{name}_joint" type="revolute"><parent>chassis</parent><child>{name}</child><pose relative_to="{name}">0 0 0 0 0 0</pose><axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper><effort>0.10</effort><velocity>25</velocity></limit><dynamics><damping>0.001</damping></dynamics></axis></joint>'''
    # PID *force* mode, not direct model translation. cmd_max/min N*m are provisional.
    plugin = f'''<plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController"><joint_name>{name}_joint</joint_name><use_force_commands>true</use_force_commands><p_gain>0.02</p_gain><i_gain>0.001</i_gain><d_gain>0</d_gain><i_max>0.05</i_max><i_min>-0.05</i_min><cmd_max>0.10</cmd_max><cmd_min>-0.10</cmd_min></plugin>'''
    return link,joint,plugin


def generate(side=.18, mass=.15, radius=.027, wheel_width=.016, wheel_offset=.078):
    if not (0.1<=side<=.3048 and 0.03<=mass<=1.0 and 0.012<=radius<=0.06 and 0.008<=wheel_width<=0.04 and .03<=wheel_offset<=.15):
        raise ValueError('Dimensions/mass outside prototype sanity bounds')
    wheel_mass = mass * .09
    chassis_mass = mass-3*wheel_mass
    z = radius + .052
    triangle_R = side/math.sqrt(3)
    # Collision approximation: central body cylinder placed above wheel tops.
    chassis_rad = min(side*.25,.054)
    chassis_h = .025
    chassis_I = inertia_cylinder(chassis_mass,chassis_rad,chassis_h)
    core = f'<link name="chassis">{inertial(chassis_mass,chassis_I, f'0 0 {fmt(z)} 0 0 0')}'
    core += visual('center_body', f'0 0 {fmt(z)} 0 0 0','cylinder',(chassis_rad,chassis_h),'0.82 0.12 0.14 1')
    core += f'<collision name="body_contact"><pose>0 0 {fmt(z)} 0 0 0</pose><geometry>{geometry("cylinder",(chassis_rad,chassis_h))}</geometry></collision>'
    # Three side beams visually mark equilateral triangle. Beam collisions omitted in v1.
    for i, theta_deg in enumerate((30,150,270)):
        theta=math.radians(theta_deg)
        mid=side*math.sqrt(3)/6
        xp,yp=mid*math.cos(theta),mid*math.sin(theta)
        core+=visual(f'edge_{i}',f'{fmt(xp)} {fmt(yp)} {fmt(z)} 0 0 {fmt(theta+math.pi/2)}','box',(side,.013,.013),'0.89 0.15 0.16 1')
    core+='</link>'
    wheels=[wheel_section(i,math.radians(a),wheel_offset,radius,wheel_width,wheel_mass) for i,a in enumerate((30,150,270))]
    metadata=f'<!-- Provisional: triangle side={fmt(side)} m; total model link mass={fmt(mass)} kg; wheel radius={fmt(radius)} m. -->'
    xml='''<?xml version="1.0"?>\n<sdf version="1.9"><model name="payload"><static>false</static><self_collide>false</self_collide>'''+metadata+core+''.join(w[0] for w in wheels)+''.join(w[1] for w in wheels)+''.join(w[2] for w in wheels)+'''<plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"><joint_name>wheel_0_joint</joint_name><joint_name>wheel_1_joint</joint_name><joint_name>wheel_2_joint</joint_name></plugin></model></sdf>'''
    ET.fromstring(xml)
    from xml.dom import minidom
    pretty=minidom.parseString(xml).toprettyxml(indent='  ')
    # strip minidom whitespace only blank lines
    OUT.parent.mkdir(parents=True,exist_ok=True)
    OUT.write_text('\n'.join(line for line in pretty.splitlines() if line.strip())+'\n')
    return OUT


if __name__ == '__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--side',type=float,default=.18)
    p.add_argument('--mass',type=float,default=.15)
    p.add_argument('--wheel-radius',type=float,default=.027)
    p.add_argument('--wheel-width',type=float,default=.016)
    p.add_argument('--wheel-offset',type=float,default=.078)
    a=p.parse_args()
    print(generate(a.side,a.mass,a.wheel_radius,a.wheel_width,a.wheel_offset))
