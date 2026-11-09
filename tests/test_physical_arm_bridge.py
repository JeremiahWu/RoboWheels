"""Guard checks use a fake bus; this test never opens a physical serial port."""
import importlib.util
import json
from pathlib import Path
import time
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_ws/src/robowheels'))
from robowheels.arms import mac_bridge as module
model_spec = importlib.util.spec_from_file_location('physical_arm_model', ROOT/'tools/cad/physical_arm_model.py')
model_module = importlib.util.module_from_spec(model_spec)
model_spec.loader.exec_module(model_module)


class Guards(unittest.TestCase):
    def setUp(self):
        config = json.loads((ROOT/'ros2_ws/src/robowheels/config/arms/servo_bridge.example.json').read_text())
        zero = {'joints': {f'{"right" if sid<=4 else "left"}_joint_{sid if sid<=4 else sid-4}': {'servo_id': sid, 'zero_raw': 2000} for sid in range(1, 9)}}
        self.bridge = module.ServoBridge(config, zero, None)
        self.bridge.items[3]["enabled"] = False
        self.writes = []
        self.bridge.read = lambda sid, addr, n: 1 if addr == 0x28 else 2000
        self.bridge.write_position = lambda *args: self.writes.append(args)
        for item in self.bridge.items.values():
            item.update(raw=2000, stamp=time.monotonic(), lo=0, hi=3900, torque=1)

    def jog(self, sid=1, delta=20):
        return self.bridge.command({'action': 'jog', 'id': sid, 'delta_raw': delta})

    def test_uncalibrated_template_refuses_to_initialize(self):
        config = json.loads((ROOT/'ros2_ws/src/robowheels/config/arms/servo_bridge.example.json').read_text())
        zeros = json.loads((ROOT/'ros2_ws/src/robowheels/config/arms/servo_zero_offsets.example.json').read_text())
        with self.assertRaisesRegex(ValueError, 'calibrate'):
            module.ServoBridge(config, zeros, None)

    def test_disarmed_and_locked_joint_reject_without_writes(self):
        with self.assertRaises(ValueError): self.jog()
        self.bridge.command({'action': 'arm'})
        with self.assertRaises(ValueError): self.jog(3)
        self.assertEqual(self.writes, [])

    def test_oversized_step_and_invalid_types(self):
        self.bridge.command({'action': 'arm'})
        for value in [41, -41, 0, 1.5, True]:
            with self.assertRaises(ValueError): self.jog(delta=value)
        self.assertEqual(self.writes, [])

    def test_stale_and_faulted_feedback_reject(self):
        self.bridge.items[1]['stamp'] = time.monotonic() - 10
        with self.assertRaises(ValueError): self.bridge.command({'action': 'arm'})
        self.bridge.items[1].update(stamp=time.monotonic(), fault='jump')
        with self.assertRaises(ValueError): self.bridge.command({'action': 'arm'})
        self.assertEqual(self.writes, [])

    def test_envelope_and_torque_off_reject(self):
        self.bridge.command({'action': 'arm'})
        self.bridge.items[1]['zero_raw'] = 1800
        with self.assertRaises(ValueError): self.jog()
        self.bridge.items[1]['zero_raw'] = 2000
        self.bridge.read = lambda sid, addr, n: 0 if addr == 0x28 else 2000
        with self.assertRaises(ValueError): self.jog()
        self.assertEqual(self.writes, [])

    def test_one_active_joint_and_hold(self):
        self.bridge.command({'action': 'arm'})
        result = self.jog(delta=-20)
        self.assertEqual(result['target_raw'], 1980)
        self.assertEqual(self.writes, [(1, 1980, 2500)])
        with self.assertRaises(ValueError): self.jog(2)
        self.bridge.command({'action': 'disarm'})
        self.assertEqual(self.writes[-1], (1, 2000, 300))
        self.assertFalse(self.bridge.armed)
        self.assertIsNone(self.bridge.active_id)

    def test_optional_per_joint_step_and_envelope(self):
        self.bridge.items[3]['enabled'] = True
        self.bridge.items[3].update(max_step_raw=20, max_offset_raw=40)
        self.bridge.command({'action': 'arm'})
        with self.assertRaises(ValueError): self.jog(3, 21)
        self.bridge.items[3]['zero_raw'] = 1950
        with self.assertRaises(ValueError): self.jog(3, 20)
        self.assertEqual(self.writes, [])
        self.bridge.items[3]['zero_raw'] = 2000
        self.jog(3, 20)
        self.assertEqual(self.writes, [(3, 2020, 2500)])

    def test_snapshot_preserves_physical_mapping(self):
        state = self.bridge.snapshot()
        self.assertEqual(state['joints'][0]['name'], 'right_shoulder_pitch')
        self.assertEqual(state['joints'][1]['name'], 'right_shoulder_roll')
        self.assertEqual(state['joints'][0]['radians'], 0.)
        self.assertFalse(state['joints'][2]['enabled'])

    def test_model_has_eight_movable_arm_joints_and_no_mock_hardware(self):
        import xml.etree.ElementTree as ET
        source = (ROOT/'ros2_ws/src/robowheels_description/urdf/robot_control.urdf').read_text()
        root = ET.fromstring(model_module.model(source))
        self.assertEqual(root.findall('ros2_control'), [])
        arms = [j for j in root.findall('joint') if j.get('type') == 'revolute']
        self.assertEqual(len(arms), 8)
        for side in ['left', 'right']:
            self.assertEqual(root.find(f"joint[@name='{side}_wrist_roll']").get('type'), 'fixed')


if __name__ == '__main__':
    unittest.main()
