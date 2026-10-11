"""Exercise real protocol disconnects and descriptor cleanup without hardware."""
import fcntl
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

RUNTIME = Path(__file__).resolve().parents[1] / 'runtime'


class ConnectionWatchTests(unittest.TestCase):
    def start_endpoint(self, lock_path):
        code = '''
import fcntl, sys, time
sys.path.insert(0, sys.argv[1])
from connection_watch import watch_connection
with watch_connection():
    lock = open(sys.argv[2], 'w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    print('READY', flush=True)
    time.sleep(60)  # Simulate a blocked initial-pose wait.
'''
        child = subprocess.Popen([sys.executable, '-u', '-c', code, str(RUNTIME), str(lock_path)],
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE)
        self.addCleanup(self.clean_child, child)
        self.assertEqual(child.stdout.readline(), b'READY\n')
        return child

    @staticmethod
    def clean_child(child):
        if child.poll() is None:
            child.kill()
        child.wait()
        for stream in (child.stdin, child.stdout, child.stderr):
            stream.close()

    def test_input_close_stops_blocked_endpoint_and_releases_lock(self):
        with tempfile.TemporaryDirectory() as tmp:
            lock_path = Path(tmp) / 'serial.lock'
            child = self.start_endpoint(lock_path)
            child.stdin.close()
            self.assertEqual(child.wait(timeout=3), 130)
            self.assertIn(b'without motor or torque commands', child.stderr.read())
            with lock_path.open('w') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def test_output_disconnect_also_stops_endpoint(self):
        with tempfile.TemporaryDirectory() as tmp:
            child = self.start_endpoint(Path(tmp) / 'serial.lock')
            child.stdout.close()
            self.assertEqual(child.wait(timeout=3), 130)

    def test_prediction_data_is_not_consumed_or_treated_as_disconnect(self):
        code = '''
import sys, time
sys.path.insert(0, sys.argv[1])
from connection_watch import watch_connection
with watch_connection():
    time.sleep(0.3)
    print(sys.stdin.readline().strip(), flush=True)
'''
        child = subprocess.Popen([sys.executable, '-u', '-c', code, str(RUNTIME)],
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE)
        self.addCleanup(self.clean_child, child)
        child.stdin.write(b'{"actions": [[2000]]}\n')
        child.stdin.flush()
        self.assertEqual(child.stdout.readline(), b'{"actions": [[2000]]}\n')
        self.assertEqual(child.wait(timeout=3), 0)
        self.assertEqual(child.stderr.read(), b'')


if __name__ == '__main__':
    unittest.main()
