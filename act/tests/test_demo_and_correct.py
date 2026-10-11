import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    'demo_and_correct', Path(__file__).parents[1] / 'training/demo_and_correct.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class DemoCorrectionTests(unittest.TestCase):
    def test_default_is_50_steps_shorter_and_hands_over_same_run(self):
        with patch.object(sys, 'argv', ['demo_and_correct']), \
                patch.object(sys.stdin, 'isatty', return_value=True), \
                patch.object(runner.subprocess, 'run') as run:
            runner.main()
        policy, correction = run.call_args_list
        args = policy.args[0]
        self.assertEqual(args[args.index('--steps')+1], '342')
        output = Path(args[args.index('--output')+1])
        self.assertEqual(correction.args[0][:3], ['ssh', '-t', 'pi'])
        self.assertIn('--source-run '+output.name, correction.args[0][3])
        self.assertTrue(policy.kwargs['check'])

    def test_failed_or_interrupted_demo_never_starts_correction(self):
        for failure in (subprocess.CalledProcessError(1, 'policy'), KeyboardInterrupt()):
            with patch.object(sys, 'argv', ['demo_and_correct']), \
                    patch.object(sys.stdin, 'isatty', return_value=True), \
                    patch.object(runner.subprocess, 'run', side_effect=failure) as run:
                with self.assertRaises(type(failure)):
                    runner.main()
                self.assertEqual(run.call_count, 1)

    def test_host_step_and_output_overrides_reach_both_processes(self):
        with patch.object(sys, 'argv', ['demo_and_correct', '--host', 'test-pi',
                                      '--steps', '250', '--output', '/tmp/attempt 1']), \
                patch.object(sys.stdin, 'isatty', return_value=True), \
                patch.object(runner.subprocess, 'run') as run:
            runner.main()
        args = run.call_args_list[0].args[0]
        self.assertEqual(args[args.index('--steps')+1], '250')
        self.assertEqual(run.call_args_list[1].args[0][:3], ['ssh', '-t', 'test-pi'])
        self.assertIn("--source-run 'attempt 1'", run.call_args_list[1].args[0][3])


if __name__ == '__main__':
    unittest.main()
