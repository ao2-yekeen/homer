import importlib.util
import json
import gzip
from pathlib import Path
import tempfile
import unittest
import sys

import numpy as np
import yaml
from unittest.mock import patch

from test_record import FakePacket, record

spec = importlib.util.spec_from_file_location('assisted', Path(__file__).parents[1]/'runtime/assisted.py')
assisted = importlib.util.module_from_spec(spec)
spec.loader.exec_module(assisted)


class FakeJoystick:
    def __init__(self):
        self.axes = [0.] * 6
        self.held = False
        self.stop = False
        self.closed = False
        self.disconnected = False

    def snapshot(self):
        if self.disconnected:
            raise OSError('controller disconnected')
        return self.axes, self.held, self.stop

    def close(self):
        self.closed = True


class AssistedTests(unittest.TestCase):
    def setUp(self):
        self.cfg = {'servo_ids': list(range(1, 7)), 'joint_names': ['j'+str(i) for i in range(6)]}
        self.packet = FakePacket([1000]*6)
        self.joy = FakeJoystick()
        self.limits = [(0, 4095)]*6

    def control(self, prefix=None):
        return assisted.AssistedControl(self.packet, 40, self.cfg, self.joy,
                                        self.limits, record.checked, record.set_torque, prefix)

    def test_replay_holds_until_release_then_leaves_manual_movement_alone(self):
        prefix = dict(positions=[[1000]*6, [1005]*6], seconds=[0., .1])
        control = assisted.AssistedControl(self.packet, 40, self.cfg, None,
                                          self.limits, record.checked, record.set_torque, prefix)
        control.begin(self.packet.positions, 0.)
        for now in (.1, .2, .3):
            control.update(list(self.packet.positions), now)
        self.assertEqual(control.phase, 'holding')
        self.assertEqual(list(self.packet.torque.values()), [1]*6)
        writes = self.packet.position_writes
        for now in (1., 30., 300.):
            control.update(list(self.packet.positions), now)
        self.assertEqual(control.phase, 'holding')
        self.assertEqual(list(self.packet.torque.values()), [1]*6)
        self.assertEqual(self.packet.position_writes, writes)
        control.release_for_hand()
        self.assertEqual(control.phase, 'hand')
        self.assertEqual(list(self.packet.torque.values()), [0]*6)
        writes = self.packet.position_writes
        self.packet.positions = [1100]*6
        control.update(list(self.packet.positions), .4)
        control.finish(self.packet.positions)
        control.close()
        self.assertEqual(self.packet.position_writes, writes)
        self.assertEqual(self.packet.positions, [1100]*6)

    def test_handoff_requires_release_and_neutral_then_deadman_moves(self):
        prefix = dict(positions=[[1000]*6, [1005]*6], seconds=[0., .1])
        control = self.control(prefix)
        self.joy.held = True
        self.joy.axes[0] = 1
        control.begin(self.packet.positions, 0.)
        for now in (.1, .2, .3):
            control.update(list(self.packet.positions), now)
        self.assertEqual(control.phase, 'joystick')
        self.assertFalse(control.ready)
        self.assertEqual(self.packet.positions, [1005]*6)
        self.joy.held = False
        self.joy.axes[0] = 0
        control.update(list(self.packet.positions), .4)
        self.assertTrue(control.ready)
        self.joy.held = True
        self.joy.axes[0] = 1
        control.update(list(self.packet.positions), .5)
        self.assertEqual(self.packet.positions, [1010]+[1005]*5)
        # Simulate physical lag: release must cancel the outstanding goal.
        self.packet.positions[0] = 1007
        self.joy.held = False
        control.update(list(self.packet.positions), .6)
        self.assertEqual(control.target[0], 1007)
        self.assertEqual(list(self.packet.torque.values()), [1]*6)
        control.close()
        self.assertEqual(list(self.packet.torque.values()), [0]*6)

    def test_disconnect_and_partial_enable_failure_release_all_torque(self):
        control = self.control()
        try:
            control.begin(self.packet.positions, 0.)
            self.joy.disconnected = True
            with self.assertRaises(OSError):
                control.update(list(self.packet.positions), .1)
        finally:
            control.close()
        self.assertEqual(list(self.packet.torque.values()), [0]*6)

        self.assertTrue(self.joy.closed)
        control = self.control()
        original = control.set_torque
        def fail_enable(packet, register, ids, enabled):
            if enabled:
                packet.torque[1] = 1
                raise RuntimeError('partial enable')
            return original(packet, register, ids, enabled)
        control.set_torque = fail_enable
        try:
            with self.assertRaisesRegex(RuntimeError, 'partial enable'):
                control.begin(self.packet.positions, 0.)
        finally:
            control.close()
        self.assertEqual(list(self.packet.torque.values()), [0]*6)

    def test_existing_powered_hold_is_staged_without_torque_toggle(self):
        self.packet.torque = {i: 1 for i in range(1, 7)}
        control = self.control()
        control.adopt_powered_hold([1000]*6)
        self.assertEqual(control.phase, 'idle')
        self.assertTrue(all(event[0] == 'position' for event in self.packet.events))
        self.assertEqual(self.packet.positions, [1000]*6)
        control.close()
        self.assertEqual(list(self.packet.torque.values()), [0]*6)

    def test_pause_does_not_resume_and_limits_bound_joystick(self):
        control = self.control()
        control.limits = [(900, 1002)]*6
        control.begin(list(self.packet.positions), 0.)
        control.update(list(self.packet.positions), .1)
        self.joy.held = True
        self.joy.axes = [1.]*6
        control.update(list(self.packet.positions), .2)
        self.assertEqual(self.packet.positions, [1002]*6)
        self.joy.stop = True
        control.update(list(self.packet.positions), .3)
        self.assertEqual(control.phase, 'paused')
        self.joy.stop = False
        self.joy.axes = [-1.]*6
        control.update(list(self.packet.positions), .4)
        self.assertEqual(self.packet.positions, [1002]*6)
        control.close()

    def test_r1_alone_does_not_send_position_goals(self):
        control = self.control()
        control.begin(list(self.packet.positions), 0.)
        control.update(list(self.packet.positions), .1)  # release and centre to arm
        self.assertTrue(control.ready)
        writes_after_begin = self.packet.position_writes

        self.joy.held = True
        control.update(list(self.packet.positions), .2)
        self.assertEqual(self.packet.position_writes, writes_after_begin)
        self.assertEqual(self.packet.positions, [1000] * 6)

        self.joy.axes[0] = .2  # button-induced axis noise stays inside deadzone
        control.update(list(self.packet.positions), .3)
        self.assertEqual(self.packet.position_writes, writes_after_begin)
        self.assertEqual(self.packet.positions, [1000] * 6)
        control.close()

    def test_feedback_lag_does_not_abort_joystick(self):
        control = self.control()
        control.begin(list(self.packet.positions), 0.)
        self.packet.positions = [900]*6
        control.update(list(self.packet.positions), .1)
        self.assertEqual(control.phase, 'joystick')
        self.joy.held = True
        self.joy.axes[0] = 1
        control.update(list(self.packet.positions), .2)
        self.assertEqual(control.target, [905]+[1000]*5)
        control.close()

    def test_moving_other_joint_does_not_walk_joint_two_hold_target(self):
        control = self.control()
        control.begin(list(self.packet.positions), 0.)
        control.update(list(self.packet.positions), .1)
        self.joy.held = True
        self.joy.axes[0] = 1
        for now, shoulder in ((.2, 990), (.3, 980), (.4, 970)):
            measured = list(self.packet.positions)
            measured[1] = shoulder
            control.update(measured, now)
            self.assertEqual(control.target[1], 1000)
        self.joy.held = False
        measured = list(self.packet.positions)
        measured[1] = 960
        control.update(measured, .5)
        self.assertEqual(control.target[1], 1000)
        control.close()

    def test_centring_joint_two_stops_it_while_r1_remains_held(self):
        control = self.control()
        control.begin(list(self.packet.positions), 0.)
        control.update(list(self.packet.positions), .1)
        self.joy.held = True
        self.joy.axes[1] = 1
        control.update(list(self.packet.positions), .2)
        self.assertEqual(control.target[1], 1005)
        self.joy.axes[1] = 0
        measured = [1000, 1002, 1000, 1000, 1000, 1000]
        control.update(measured, .3)
        self.assertEqual(control.target[1], 1002)
        measured[1] = 990
        self.joy.axes[0] = 1
        control.update(measured, .4)
        self.assertEqual(control.target[1], 1002)
        control.close()

    def test_replay_stall_times_out(self):
        control = self.control(dict(positions=[[1000]*6, [1040]*6], seconds=[0., .1]))
        control.begin(list(self.packet.positions), 0.)
        self.packet.positions = [900]*6
        control.update(list(self.packet.positions), .1)
        self.assertEqual(control.phase, 'replay')
        self.packet.positions = [1000]*6
        control.update(list(self.packet.positions), .2)
        self.packet.positions = [900]*6
        with self.assertRaisesRegex(RuntimeError, 'stalled'):
            control.update(list(self.packet.positions), 21.)
        control.close()

    def test_joint_two_targets_keep_advancing_both_ways_despite_feedback_drift(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                control = self.control()
                control.begin([1000]*6, 0.)
                self.joy.held = False
                self.joy.axes = [0.]*6
                control.update([1000]*6, .1)
                self.joy.held = True
                self.joy.axes[1] = direction
                for frame in range(1, 6):
                    measured = [1000]*6
                    measured[1] -= direction * (frame-1)*10
                    control.update(measured, .1 + frame*.1)
                    self.assertEqual(control.target[1], 1000 + direction*frame*5)
                # Reversing cancels the old outstanding target immediately.
                self.joy.axes[1] = -direction
                control.update([1000]*6, .7)
                self.assertEqual(control.target[1], 1000 - direction*5)
                control.close()

    def test_small_joint_two_input_accumulates_fractional_steps(self):
        control = self.control()
        control.begin([1000]*6, 0.)
        control.update([1000]*6, .1)
        self.joy.held = True
        self.joy.axes[1] = -.3
        for frame in range(1, 11):
            control.update([1000]*6, .1 + frame*.1)
        self.assertEqual(control.target[1], 997)
        control.close()

    def test_replay_sends_first_bounded_goal_before_waiting_for_feedback(self):
        # A large first source movement used to wait for the arm to move before
        # issuing any goal, leaving the replay permanently stationary.
        control = self.control(dict(positions=[[1000]*6, [1400]*6],
                                    seconds=[0., .1]))
        control.begin(list(self.packet.positions), 0.)
        control.update(list(self.packet.positions), .1)
        self.assertEqual(self.packet.positions, [1100]*6)
        self.assertEqual(control.replay_time, 0.)
        control.close()

    def test_replay_sends_final_step_before_waiting_for_feedback(self):
        control = self.control(dict(positions=[[1000]*6, [1200]*6],
                                    seconds=[0., .1]))
        control.begin(list(self.packet.positions), 0.)
        control.update([1000]*6, .2)
        self.assertEqual(control.target, [1100]*6)
        control.update([1000]*6, .4)
        self.assertEqual(control.target, [1200]*6)
        self.assertEqual(control.replay_time, 0.)
        control.update([1200]*6, .7)
        self.assertEqual(control.phase, 'joystick')
        control.close()

    def test_source_requires_success_marker_matching_start_and_valid_limits(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            meta = dict(outcome='success', config=self.cfg, approach_end_frame=1)
            (directory/'metadata.json').write_text(json.dumps(meta))
            rows = [dict(state_ticks=[1000]*6, read_end_ns=100000000),
                    dict(state_ticks=[1005]*6, read_end_ns=200000000)]
            (directory/'samples.jsonl').write_text('\n'.join(map(json.dumps, rows)))
            prefix = assisted.load_prefix(directory, None, self.cfg, self.limits, [1000]*6)
            self.assertEqual(prefix['positions'][-1], [1005]*6)
            with gzip.open(directory/'samples.jsonl.gz', 'wt') as stream:
                stream.write('\n'.join(map(json.dumps, rows)))
            (directory/'samples.jsonl').unlink()
            self.assertEqual(assisted.load_prefix(directory, None, self.cfg,
                                                 self.limits, [1000]*6), prefix)
            with self.assertRaisesRegex(RuntimeError, 'different session'):
                assisted.load_prefix(directory, None, self.cfg, self.limits, [1100]*6)
            with self.assertRaisesRegex(RuntimeError, 'travel limits'):
                assisted.load_prefix(directory, None, self.cfg, [(0, 1002)]*6, [1000]*6)
            with self.assertRaisesRegex(RuntimeError, 'outside the source'):
                assisted.load_prefix(directory, 9, self.cfg, self.limits, [1000]*6)
            meta['approach_end_frame'] = None
            (directory/'metadata.json').write_text(json.dumps(meta))
            with self.assertRaisesRegex(RuntimeError, 'Mark the approach'):
                assisted.load_prefix(directory, None, self.cfg, self.limits, [1000]*6)
            meta['outcome'] = 'discard'
            (directory/'metadata.json').write_text(json.dumps(meta))
            with self.assertRaisesRegex(RuntimeError, 'successful episode'):
                assisted.load_prefix(directory, 1, self.cfg, self.limits, [1000]*6)


class AssistedRecorderIntegrationTests(unittest.TestCase):
    def test_replay_and_hand_guidance_are_one_fresh_recorded_episode(self):
        self.run_recording()

    def test_ctrl_c_during_replay_returns_to_start_and_releases_torque(self):
        self.run_recording(interrupt_phase='replay')

    def test_ctrl_c_during_hand_guidance_returns_to_start_and_releases_torque(self):
        self.run_recording(interrupt_phase='hand')

    def test_ctrl_c_while_holding_returns_to_start_and_releases_torque(self):
        self.run_recording(interrupt_phase='holding')

    def run_recording(self, interrupt_phase=None):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'data').mkdir()
            cfg = dict(servo_ids=list(range(1, 7)), joint_names=['j'+str(i) for i in range(6)],
                       camera='wrist', width=20, height=20, fps=10,
                       overview_camera=dict(device='overview', width=20, height=20),
                       joystick=dict(device='fake'))
            config_path = root/'rig.yaml'
            config_path.write_text(yaml.safe_dump(cfg))
            (root/'data/session-start-pose.json').write_text(json.dumps(
                dict(joint_names=cfg['joint_names'], state_ticks=[1000]*6)))
            packet = FakePacket([1000]*6)
            joystick = FakeJoystick()
            prefix = dict(source='/source', until_frame=1,
                          positions=[[1000]*6, [1005]*6], seconds=[0., .1])
            controls = []
            class Camera:
                def __init__(self, *args):
                    self.count = 0
                def set(self, *args):
                    pass
                def isOpened(self):
                    return True
                def read(self):
                    self.count += 1
                    return True, np.full((20, 20, 3), self.count, dtype=np.uint8)
                def release(self):
                    pass
            class Handle:
                closed = False
                def close(self):
                    self.closed = True
                def closePort(self):
                    self.closed = True
            port, lock = Handle(), Handle()
            class Clock:
                now = 0.
                def monotonic(self):
                    self.now += .01
                    return self.now
                def monotonic_ns(self):
                    return int(self.monotonic()*1e9)
            clock = Clock()
            keys = dict(started=False, marked=False, saved=False)
            def key(*args):
                if not keys['started']:
                    keys['started'] = True
                    return ord('s')
                if keys['saved']:
                    return ord('q')
                if controls and controls[0].phase == interrupt_phase:
                    if interrupt_phase == 'hand':
                        packet.positions[1] = 1100
                    raise KeyboardInterrupt
                if controls and controls[0].phase == 'holding':
                    self.assertEqual(list(packet.torque.values()), [1]*6)
                    return ord('h')
                if controls and controls[0].phase == 'hand':
                    self.assertEqual(list(packet.torque.values()), [0]*6)
                    packet.positions[1] = 1100
                    if not keys['marked']:
                        keys['marked'] = True
                        return ord('p')
                    keys['saved'] = True
                    return ord('f')
                if clock.now > 5:
                    raise AssertionError('Replay failed to hand off')
                return -1
            # Exercise main, cameras, source replay, marker, save and cleanup together.
            from contextlib import ExitStack
            with ExitStack() as stack:
                stack.enter_context(patch.dict(sys.modules, assisted=assisted))
                stack.enter_context(patch.object(assisted, 'XboxJoystick', side_effect=AssertionError('Manual replay must not open joystick')))
                stack.enter_context(patch.object(assisted, 'load_prefix', return_value=prefix))
                # Save the real class because the factory itself uses the module.
                real_control = assisted.AssistedControl
                def factory(*args, **kwargs):
                    control = real_control(*args, **kwargs)
                    controls.append(control)
                    return control
                stack.enter_context(patch.object(assisted, 'AssistedControl', side_effect=factory))
                stack.enter_context(patch.object(record, 'ROOT', root))
                stack.enter_context(patch.object(record, 'open_arm', return_value=(port, packet, 40, lock)))
                returns = stack.enter_context(patch.object(record, 'return_to_pose', wraps=record.return_to_pose))
                stack.enter_context(patch.object(record.cv2, 'VideoCapture', Camera))
                stack.enter_context(patch.object(record.cv2, 'imshow'))
                stack.enter_context(patch.object(record.cv2, 'destroyAllWindows'))
                stack.enter_context(patch.object(record.cv2, 'waitKey', side_effect=key))
                stack.enter_context(patch.object(record.time, 'monotonic', side_effect=clock.monotonic))
                stack.enter_context(patch.object(record.time, 'monotonic_ns', side_effect=clock.monotonic_ns))
                stack.enter_context(patch.object(record.time, 'sleep'))
                stack.enter_context(patch.object(sys, 'argv', ['record.py', '--config', str(config_path), '--replay', '/source']))
                record.main()
            episode, = (root/'data').glob('episode-*')
            metadata = json.loads((episode/'metadata.json').read_text())
            with gzip.open(episode/'samples.jsonl.gz', 'rt') as stream:
                rows = [json.loads(line) for line in stream]
            self.assertTrue(port.closed)
            self.assertTrue(lock.closed)
            if interrupt_phase:
                self.assertEqual(metadata['outcome'], 'invalid')
                self.assertEqual(returns.call_count, 2)
                self.assertEqual(packet.positions, [1000]*6)
                self.assertEqual(list(packet.torque.values()), [0]*6)
                return
            self.assertEqual(metadata['outcome'], 'success')
            self.assertEqual(metadata['assistance']['replay_source'], '/source')
            self.assertIsNotNone(metadata['approach_end_frame'])
            self.assertIn('replay', {row['control']['phase'] for row in rows})
            self.assertIn('holding', {row['control']['phase'] for row in rows})
            self.assertIn('hand', {row['control']['phase'] for row in rows})
            self.assertTrue(any(row['state_ticks'][1] == 1100 for row in rows))
            self.assertEqual(metadata['assistance']['mode'], 'replay-then-hand')
            pixels = [int(record.cv2.imread(str(episode/row['image']))[15, 15, 0]) for row in rows]
            self.assertEqual(len(pixels), len(set(pixels)))
            self.assertEqual(list(packet.torque.values()), [0]*6)
            self.assertFalse(joystick.closed)


if __name__ == '__main__':
    unittest.main()
