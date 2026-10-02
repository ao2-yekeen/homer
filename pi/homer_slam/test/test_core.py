from pathlib import Path
from types import SimpleNamespace
import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml
from homer_slam.core import load_geometry, map_prefix, age_seconds, freshness
from homer_slam.model import model_geometry, display_description
from homer_slam.network import render

PACKAGE = Path(__file__).resolve().parents[1]
URDF = PACKAGE.parents[1]/'simulation/urdf/home_mobile_manipulator_v6_real_base_layout.urdf'


class GeometryTests(unittest.TestCase):
    def test_model_base_and_offset(self):
        geometry = model_geometry(URDF)
        self.assertEqual(geometry['model_center'], [0.,0.,0.])
        self.assertAlmostEqual(geometry['laser']['x'], .09)
        self.assertAlmostEqual(geometry['laser']['y'], .075)
        self.assertAlmostEqual(geometry['laser']['z'], .093)
        self.assertEqual(geometry['footprint'], [[-.159,-.175],[.159,-.175],[.159,.175],[-.159,.175]])

    def test_geometry_rejects_unverified_and_invalid(self):
        with self.assertRaisesRegex(ValueError, 'unverified'):
            load_geometry(PACKAGE/'config/geometry.yaml')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'geometry.yaml'
            geometry = model_geometry(URDF)
            geometry['laser']['x'] = float('nan')
            path.write_text(yaml.safe_dump(geometry))
            with self.assertRaisesRegex(ValueError, 'laser.x'): load_geometry(path)
            geometry['laser']['x'] = .09
            geometry['footprint'] = [[0,0],[1,0],[2,0]]
            path.write_text(yaml.safe_dump(geometry))
            with self.assertRaisesRegex(ValueError, 'zero area'): load_geometry(path)

    def test_display_model_has_no_fake_moving_joints(self):
        root = ET.fromstring(display_description(URDF))
        self.assertTrue(all(j.get('type') == 'fixed' for j in root.findall('joint')))
        self.assertTrue(any(j.get('type') != 'fixed' for j in ET.parse(URDF).getroot().findall('joint')))

    def test_both_serialized_files_required(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix = Path(directory)/'map'
            with self.assertRaises(ValueError): map_prefix(prefix)
            Path(str(prefix)+'.posegraph').touch()
            with self.assertRaises(ValueError): map_prefix(prefix)
            Path(str(prefix)+'.data').touch()
            self.assertEqual(map_prefix(str(prefix)+'.posegraph'), prefix)

    def test_timestamp_age_and_future_clock(self):
        self.assertAlmostEqual(age_seconds(2_100_000_000, SimpleNamespace(sec=2,nanosec=0)), .1)
        self.assertEqual(freshness(.6,.5),'stale')
        self.assertEqual(freshness(None,.5),'missing')
        self.assertIn('future',freshness(-.1,.5))
        self.assertEqual(freshness(-.2,.5,.25),'fresh')
        self.assertIn('future',freshness(-.3,.5,.25))


class NetworkTests(unittest.TestCase):
    def test_profiles_do_not_mix_peers(self):
        config = json.loads((PACKAGE/'config/network.json').read_text())
        hotspot = render(config,'hotspot','pi')
        tailscale = render(config,'tailscale','pi')
        self.assertIn('192.168.137.250',hotspot)
        self.assertNotIn('100.66.192.19',hotspot)
        self.assertIn('100.66.192.19',tailscale)
        self.assertNotIn('192.168.137.250',tailscale)
        self.assertIn('<AllowMulticast>false</AllowMulticast>',hotspot)
        with self.assertRaises(ValueError): render(config,'hotspot','laptop')
        with self.assertRaises(ValueError): render(config,'hotspot','pi',local_address='172.30.1.1')
