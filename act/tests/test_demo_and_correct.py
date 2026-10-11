import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    'demo_and_correct', Path(__file__).parents[1] / 'training/demo_and_correct.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class DemoCorrectionTests(unittest.TestCase):
    def source(self, directory):
        (directory / 'movement-report.json').write_text(json.dumps(dict(
            commands=[dict(measured_ticks=[1000+step]*6, sent_ticks=[1100+step]*6) for step in range(1, 343)])))
        for chunk, steps, finished in [(0, 0, False), (6, 300, False), (7, 342, True)]:
            (directory / f'{chunk:03d}-state.json').write_text(json.dumps(dict(
                state_ticks=[2000+chunk]*6, joint_names=['a']*6,
                executed_steps=steps, finished=finished)))

    def test_default_skips_policy_and_uses_feedback_twenty_steps_before_end(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp)
            self.source(source)
            with patch.object(sys, 'argv', ['demo_and_correct', '--source-run', tmp, '--steps-before-end', '20']), \
                    patch.object(sys.stdin, 'isatty', return_value=True), \
                    patch.object(runner, 'latest_completed_run', return_value=source), \
                    patch.object(runner.subprocess, 'run') as run:
                runner.main()
            upload, correction = run.call_args_list
            self.assertEqual(upload.args[0][0], 'scp')
            pose = json.loads((source / 'correction-start-pose.json').read_text())
            self.assertEqual(pose['state_ticks'], [1322]*6)
            self.assertEqual(pose['executed_steps'], 322)
            self.assertEqual(pose['initial_ticks'], [2000]*6)
            self.assertEqual(len(pose['replay_actions']), 322)
            self.assertEqual(pose['replay_actions'][-1], [1422]*6)
            self.assertEqual(pose['source_state'], 'movement-report.json')
            self.assertEqual(correction.args[0][:3], ['ssh', '-t', 'pi'])
            self.assertIn('--start-pose act/data/correction-start-pose.json', correction.args[0][3])

    def test_offset_selects_feedback_or_initial_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp)
            self.source(source)
            self.assertEqual(runner.saved_chunk_pose(source, 0)['state_ticks'], [1342]*6)
            self.assertEqual(runner.saved_chunk_pose(source, 50)['state_ticks'], [1292]*6)
            self.assertEqual(runner.saved_chunk_pose(source, 400)['state_ticks'], [2000]*6)

    def test_failed_or_interrupted_fresh_demo_never_starts_correction(self):
        for failure in (subprocess.CalledProcessError(1, 'policy'), KeyboardInterrupt()):
            with patch.object(sys, 'argv', ['demo_and_correct', '--run-act']), \
                    patch.object(sys.stdin, 'isatty', return_value=True), \
                    patch.object(runner.subprocess, 'run', side_effect=failure) as run:
                with self.assertRaises(type(failure)):
                    runner.main()
                self.assertEqual(run.call_count, 1)
                args = run.call_args.args[0]
                self.assertEqual(args[args.index('--steps')+1], '250')

    def test_each_success_runs_fresh_policy_then_records_current_pose(self):
        from unittest.mock import Mock
        with patch.object(sys, 'argv', ['demo_and_correct']), \
                patch.object(sys.stdin, 'isatty', return_value=True), \
                patch('builtins.input', return_value=''), \
                patch.object(runner.subprocess, 'run', side_effect=[
                    Mock(returncode=0), Mock(returncode=10),
                    Mock(returncode=0), Mock(returncode=0)]) as run:
            runner.main()
        calls = [call.args[0] for call in run.call_args_list]
        self.assertEqual(len(calls), 4)
        for command in calls[::2]:
            self.assertIn('250', command)
            self.assertTrue(command[2].endswith('run_policy.py'))
        for command in calls[1::2]:
            self.assertIn('--once', command[-1])
            self.assertNotIn('--start-pose', command[-1])
        self.assertNotEqual(calls[0][calls[0].index('--output')+1],
                            calls[2][calls[2].index('--output')+1])

    def test_source_and_host_overrides(self):
        with tempfile.TemporaryDirectory(prefix='attempt ') as tmp:
            source = Path(tmp)
            self.source(source)
            with patch.object(sys, 'argv', ['demo_and_correct', '--host', 'test-pi',
                                           '--source-run', tmp, '--steps-before-end', '50']), \
                    patch.object(sys.stdin, 'isatty', return_value=True), \
                    patch.object(runner.subprocess, 'run') as run:
                runner.main()
            self.assertEqual(json.loads((source / 'correction-start-pose.json').read_text())['executed_steps'], 292)
            self.assertEqual(run.call_args_list[1].args[0][:3], ['ssh', '-t', 'test-pi'])
            self.assertIn("--source-run '"+source.name+"'", run.call_args_list[1].args[0][3])


if __name__ == '__main__':
    unittest.main()
