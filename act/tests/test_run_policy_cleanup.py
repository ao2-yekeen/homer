import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, call, patch

spec = importlib.util.spec_from_file_location(
    'policy_runner_cleanup', Path(__file__).resolve().parents[1] / 'training/run_policy.py')
runner = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'predict_snapshot': MagicMock()}):
    spec.loader.exec_module(runner)


class CleanupTests(unittest.TestCase):
    def test_runner_replans_from_fresh_observation_after_five_actions(self):
        process = MagicMock()
        snapshots = [dict(images={'wrist': 'eA==', 'overview': 'eA=='},
                          state_ticks=[i] * 6, executed_steps=i, finished=i == 10)
                     for i in (0, 5, 10)]
        process.stdout = iter(json.dumps(s) + '\n' for s in snapshots)
        policy = MagicMock()
        policy.predict.side_effect = [dict(actions=[[i] * 6 for i in range(50)],
                                           fps=10, joint_names=['joint'] * 6,
                                           inference_seconds=0.2) for _ in range(2)]
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(sys, 'argv', ['run_policy.py', '--checkpoint', directory,
                                           '--output', directory, '--steps', '10']), \
                    patch.object(runner, 'SnapshotPolicy', return_value=policy), \
                    patch.object(runner.subprocess, 'run'), \
                    patch.object(runner.subprocess, 'Popen', return_value=process), \
                    patch.object(runner, 'close_session', return_value=0), \
                    patch.object(runner.Image, 'open'), patch('builtins.print'):
                runner.main()
            self.assertEqual(policy.predict.call_count, 2)
            self.assertEqual([c.args[0] for c in policy.predict.call_args_list],
                             [[0] * 6, [5] * 6])
            sent = [json.loads(c.args[0]) for c in process.stdin.write.call_args_list]
            self.assertEqual([len(p['actions']) for p in sent], [5, 5])
            self.assertEqual(sent[0]['actions'], [[i] * 6 for i in range(5)])
            saved = json.loads((Path(directory) / '000-prediction.json').read_text())
            self.assertEqual(len(saved['actions']), 50)

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
