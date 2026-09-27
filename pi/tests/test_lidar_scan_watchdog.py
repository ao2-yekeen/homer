import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lidar_scan_watchdog import MAX_SNAPSHOTS, ScanHealth, command, write_event


class ScanHealthTests(unittest.TestCase):
    def test_startup_grace_and_missing_first_scan(self):
        health = ScanHealth(0)
        self.assertEqual(health.check(59), [])
        self.assertEqual(health.check(60), [("outage", 60)])

    def test_stale_scan_and_repeated_snapshot(self):
        health = ScanHealth(0)
        self.assertEqual(health.scan(20), [("baseline", None)])
        self.assertEqual(health.check(59), [])
        self.assertEqual(health.check(60), [("outage", 40)])
        self.assertEqual(health.check(119), [])
        self.assertEqual(health.check(120), [("outage_repeat", 100)])
        self.assertEqual(health.check(121), [])

    def test_recovery_and_new_outage(self):
        health = ScanHealth(0)
        health.scan(10)
        health.check(60)
        self.assertEqual(health.scan(65), [("recovery", 5)])
        self.assertEqual(health.scan(66), [])
        self.assertEqual(health.check(80), [])
        self.assertEqual(health.check(81), [("outage", 15)])

    def test_failed_diagnostic_command_is_recorded(self):
        with patch("lidar_scan_watchdog.subprocess.run", side_effect=OSError("missing")):
            self.assertIn("missing", command(["missing-command"])["error"])

    def test_evidence_is_private_and_retained_to_limit(self):
        with tempfile.TemporaryDirectory() as root, patch("lidar_scan_watchdog.snapshot", return_value={}):
            directory = Path(root) / "evidence"
            for _ in range(MAX_SNAPSHOTS + 2):
                path = write_event(directory, "outage", 15)
            self.assertEqual(len(list(directory.glob("*.json"))), MAX_SNAPSHOTS)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)


if __name__ == "__main__":
    unittest.main()
