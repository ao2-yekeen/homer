import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import MagicMock, call, patch

spec = importlib.util.spec_from_file_location(
    'policy_runner_cleanup', Path(__file__).resolve().parents[1] / 'training/run_policy.py')
runner = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'predict_snapshot': MagicMock()}):
    spec.loader.exec_module(runner)


class CleanupTests(unittest.TestCase):
    def test_interrupt_sends_return_and_waits_before_closing_input(self):
        process = MagicMock()
        process.wait.return_value = 130
        with patch.object(runner.threading, 'Thread'):
            self.assertEqual(runner.close_session(process, return_to_start=True), 130)
        self.assertEqual(process.mock_calls, [
            call.stdin.write('{"return_to_start": true}\n'), call.stdin.flush(),
            call.wait(timeout=30), call.stdin.close()])

    def test_input_closes_before_wait(self):
        process = MagicMock()
        process.wait.return_value = 0
        self.assertEqual(runner.close_session(process), 0)
        self.assertEqual(process.mock_calls, [call.stdin.close(), call.wait(timeout=5)])

    def test_broken_pipe_still_reaps_ssh(self):
        process = MagicMock()
        process.stdin.close.side_effect = BrokenPipeError()
        process.wait.return_value = 130
        self.assertEqual(runner.close_session(process), 130)
        process.wait.assert_called_once_with(timeout=5)

    def test_unresponsive_ssh_is_terminated_then_killed_and_reaped(self):
        process = MagicMock()
        process.wait.side_effect = [subprocess.TimeoutExpired('ssh', 5),
                                    subprocess.TimeoutExpired('ssh', 2), -9]
        self.assertEqual(runner.close_session(process), -9)
        self.assertEqual(process.mock_calls, [call.stdin.close(), call.wait(timeout=5),
                                             call.terminate(), call.wait(timeout=2),
                                             call.kill(), call.wait()])


if __name__ == '__main__':
    unittest.main()
