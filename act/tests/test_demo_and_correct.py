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
        for chunk, steps, finished in [(0, 0, False), (6, 300, False), (7, 342, True)]:
            (directory / f'{chunk:03d}-state.json').write_text(json.dumps(dict(
                state_ticks=[2000+chunk]*6, joint_names=['a']*6,
                executed_steps=steps, finished=finished)))

    def test_default_skips_policy_and_uploads_measured_final_chunk_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp)
            self.source(source)
            with patch.object(sys, 'argv', ['demo_and_correct']), \
                    patch.object(sys.stdin, 'isatty', return_value=True), \
                    patch.object(runner, 'latest_completed_run', return_value=source), \
                    patch.object(runner.subprocess, 'run') as run:
                runner.main()
            upload, correction = run.call_args_list
            self.assertEqual(upload.args[0][0], 'scp')
            pose = json.loads((source / 'correction-start-pose.json').read_text())
            self.assertEqual(pose['state_ticks'], [2006]*6)
            self.assertEqual(pose['executed_steps'], 300)
            self.assertEqual(correction.args[0][:3], ['ssh', '-t', 'pi'])
            self.assertIn('--start-pose act/data/correction-start-pose.json', correction.args[0][3])

    def test_failed_or_interrupted_fresh_demo_never_starts_correction(self):
        for failure in (subprocess.CalledProcessError(1, 'policy'), KeyboardInterrupt()):
            with patch.object(sys, 'argv', ['demo_and_correct', '--run-act']), \
                    patch.object(sys.stdin, 'isatty', return_value=True), \
                    patch.object(runner.subprocess, 'run', side_effect=failure) as run:
                with self.assertRaises(type(failure)):
                    runner.main()
                self.assertEqual(run.call_count, 1)
                args = run.call_args.args[0]
                self.assertEqual(args[args.index('--steps')+1], '342')

    def test_source_and_host_overrides(self):
        with tempfile.TemporaryDirectory(prefix='attempt ') as tmp:
            source = Path(tmp)
            self.source(source)
            with patch.object(sys, 'argv', ['demo_and_correct', '--host', 'test-pi',
                                           '--source-run', tmp]), \
                    patch.object(sys.stdin, 'isatty', return_value=True), \
                    patch.object(runner.subprocess, 'run') as run:
                runner.main()
            self.assertEqual(run.call_args_list[1].args[0][:3], ['ssh', '-t', 'test-pi'])
            self.assertIn("--source-run '"+source.name+"'", run.call_args_list[1].args[0][3])


if __name__ == '__main__':
    unittest.main()
