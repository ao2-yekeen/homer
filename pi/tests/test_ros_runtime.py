import ast
import importlib.util
import io
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

PI = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PI))
from ros_runtime import configure_ros_environment, RosEnvironmentError


class RosRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.xml = self.directory / "cyclonedds.xml"
        self.xml.write_text('<CycloneDDS><Domain Id="0" /></CycloneDDS>')
        self.config = self.directory / "ros.env"
        self.config.write_text(
            f'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\nROS_DOMAIN_ID=0\n'
            f'CYCLONEDDS_URI="{self.xml}"\n'
            'ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT\n')

    def configure(self, environment=None):
        return configure_ros_environment({} if environment is None else environment, self.config)

    def test_fresh_shell_loads_full_service_configuration(self):
        environment = {"UNRELATED": "preserved"}
        self.configure(environment)
        self.assertEqual(environment["RMW_IMPLEMENTATION"], "rmw_cyclonedds_cpp")
        self.assertEqual(environment["CYCLONEDDS_URI"], str(self.xml))
        self.assertEqual(environment["ROS_DOMAIN_ID"], "0")
        self.assertEqual(environment["UNRELATED"], "preserved")

    def test_service_environment_is_accepted_without_changes(self):
        environment = self.configure()
        before = environment.copy()
        self.configure(environment)
        self.assertEqual(environment, before)

    def test_ros_setup_discovery_default_is_replaced_by_service_policy(self):
        environment = {"ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET"}
        self.configure(environment)
        self.assertEqual(environment["ROS_AUTOMATIC_DISCOVERY_RANGE"], "SYSTEM_DEFAULT")

    def test_conflicts_fail_before_changing_environment(self):
        for key, value in (("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
                           ("ROS_DOMAIN_ID", "12"), ("CYCLONEDDS_URI", "/wrong.xml")):
            with self.subTest(key=key):
                environment = {key: value}
                with self.assertRaisesRegex(RosEnvironmentError, "conflicts"):
                    self.configure(environment)
                self.assertEqual(environment, {key: value})

    def test_missing_dds_file_is_rejected(self):
        self.xml.unlink()
        with self.assertRaisesRegex(RosEnvironmentError, "Invalid CYCLONEDDS_URI"):
            self.configure()

    def test_malformed_dds_file_is_rejected(self):
        self.xml.write_text("<CycloneDDS>")
        with self.assertRaisesRegex(RosEnvironmentError, "Invalid CYCLONEDDS_URI"):
            self.configure()

    def test_partial_service_configuration_is_rejected(self):
        self.config.write_text("RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n")
        with self.assertRaisesRegex(RosEnvironmentError, "CYCLONEDDS_URI"):
            self.configure()

    def test_missing_explicit_configuration_is_rejected(self):
        self.config.unlink()
        with self.assertRaisesRegex(RosEnvironmentError, "file missing"):
            self.configure()

    def test_fastdds_override_is_rejected_even_without_service_file(self):
        with patch("ros_runtime.Path.home", return_value=self.directory):
            with self.assertRaisesRegex(RosEnvironmentError, "require rmw_cyclonedds_cpp"):
                configure_ros_environment({"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"})

    def test_plain_checkout_defaults_to_cyclone(self):
        with patch("ros_runtime.Path.home", return_value=self.directory):
            self.assertEqual(configure_ros_environment({}),
                             {"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"})

    def test_invalid_domain_is_rejected(self):
        for value in ("abc", "-1", "233"):
            with self.subTest(value=value):
                self.config.write_text(
                    f'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\nROS_DOMAIN_ID={value}\n'
                    f'CYCLONEDDS_URI={self.xml}\n')
                with self.assertRaisesRegex(RosEnvironmentError, "ROS_DOMAIN_ID"):
                    self.configure()

    def test_environment_file_is_not_executed(self):
        marker = self.directory / "must-not-exist"
        self.config.write_text(
            f'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n'
            f'CYCLONEDDS_URI=$(touch {marker})\n')
        with self.assertRaises(RosEnvironmentError):
            self.configure()
        self.assertFalse(marker.exists())

    def test_standalone_ros_tools_configure_before_ros_imports(self):
        files = list(PI.glob("*.py")) + [PI / "approach_object/tools/capture_rgb_tof.py"]
        checked = 0
        for source in files:
            tree = ast.parse(source.read_text())
            imports = [node.lineno for node in ast.walk(tree)
                       if (isinstance(node, ast.Import) and
                           any(alias.name == "rclpy" for alias in node.names))
                       or (isinstance(node, ast.ImportFrom) and
                           (node.module or "").split(".")[0] == "rclpy")]
            if not imports:
                continue
            checked += 1
            calls = [node.lineno for node in ast.walk(tree)
                     if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                     and node.func.id == "ensure_ros_environment"]
            with self.subTest(source=source.name):
                self.assertTrue(calls, "ROS tools must use the shared environment loader")
                self.assertLess(min(calls), min(imports))
        self.assertGreaterEqual(checked, 8)

    def test_capture_upload_includes_shared_runtime(self):
        source = PI / "approach_object/tools/run_named_object_distance_test.py"
        spec = importlib.util.spec_from_file_location("capture_upload_test", source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        uploaded = []

        def fake_run(args, **kwargs):
            if "stdin" in kwargs:
                archive = io.BytesIO(kwargs["stdin"].read())
                with tarfile.open(fileobj=archive) as bundle:
                    uploaded.extend(bundle.getnames())

        with patch.object(module, "run", side_effect=fake_run):
            module.capture_pi("fake-pi", self.directory / "capture", 1)
        self.assertIn("ros_runtime.py", uploaded)


if __name__ == "__main__":
    unittest.main()
