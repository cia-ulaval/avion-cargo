import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from xml.etree import ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT / 'scripts'))
from generate_payload_physics import generate, OUT
from apply_payload_to_world import install

class PayloadModelTest(unittest.TestCase):
    def test_model_three_dynamic_wheels_with_real_revolute_joints(self):
        m=ET.parse(ROOT/'simulation/models/payload/model.sdf').getroot().find('model')
        self.assertEqual(m.findtext('static'),'false')
        links=m.findall('link'); self.assertEqual(len(links),4)
        self.assertEqual(len(m.findall('joint')),3)
        self.assertEqual(len(m.findall('plugin')),4)
        mass=sum(float(l.findtext('inertial/mass')) for l in links)
        self.assertAlmostEqual(mass,.15,places=6)
        for i in range(3):
            wheel=m.find(f"./link[@name='wheel_{i}']")
            self.assertIsNotNone(wheel.find("collision/surface/friction/ode"))
            joint=m.find(f"./joint[@name='wheel_{i}_joint']")
            self.assertEqual(joint.findtext('axis/xyz'),'0 0 1')
            self.assertIsNotNone(m.find(f"./plugin[joint_name='wheel_{i}_joint']"))
            self.assertEqual(m.find(f"./plugin[joint_name='wheel_{i}_joint']").findtext('use_force_commands'),'true')
    def test_inertia_positive(self):
        root=ET.parse(ROOT/'simulation/models/payload/model.sdf').getroot()
        for l in root.findall('.//link'):
            for k in ('ixx','iyy','izz'):
                self.assertGreater(float(l.findtext('inertial/inertia/'+k)),0)
    def test_standalone_world_parseable(self):
        w=ET.parse(ROOT/'simulation/worlds/payload_physics_test.sdf').getroot()
        self.assertEqual(w.findtext('.//include/uri'),'model://payload')
    def test_installer_preserves_world_and_is_idempotent(self):
        original='''<?xml version="1.0"?><sdf version="1.9"><world name="a"><include><uri>model://tricopter_vtol</uri></include><model name="landing_marker"/></world></sdf>'''
        # Installer intentionally only accepts team's newline format.
        original=original.replace('</world></sdf>', '\n  </world>\n</sdf>')
        with TemporaryDirectory() as tmp:
            p=Path(tmp)/'world.sdf';p.write_text(original)
            self.assertTrue(install(p))
            once=p.read_text();self.assertEqual(once.count('model://payload'),1)
            self.assertFalse(install(p));self.assertEqual(p.read_text(),once)
    def test_model_config(self):
        e=ET.parse(ROOT/'simulation/models/payload/model.config').getroot()
        self.assertEqual(e.findtext('sdf'),'model.sdf')

if __name__=='__main__':unittest.main()
