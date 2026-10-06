"""Use the robot services' DDS configuration before importing ROS.

Also usable as a launcher: python3 pi/ros_runtime.py ros2 topic list
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import shlex
import sys
import xml.etree.ElementTree as ET


ROS_ENV_KEYS = frozenset({
    "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "CYCLONEDDS_URI",
    "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_STATIC_PEERS", "ROS_LOCALHOST_ONLY",
})


class RosEnvironmentError(ValueError):
    """A ROS command must not start with this configuration."""


def configure_ros_environment(environment=None, config_path=None):
    environment = os.environ if environment is None else environment
    explicit_path = config_path is not None
    config = (Path(config_path) if explicit_path else
              Path.home() / ".config/homer-slam/network/ros.env")
    settings = {}
    if explicit_path and not config.is_file():
        raise RosEnvironmentError(f"ROS environment file missing: {config}")
    if config.is_file():
        for number, line in enumerate(config.read_text().splitlines(), 1):
            key, separator, raw = line.strip().partition("=")
            if not separator or key not in ROS_ENV_KEYS:
                continue
            try:
                values = shlex.split(raw, comments=True)
            except ValueError as error:
                raise RosEnvironmentError(f"Invalid ROS setting at {config}:{number}") from error
            if len(values) != 1 or any(token in values[0] for token in ("$", "`")):
                raise RosEnvironmentError(f"ROS setting {key} must be a literal value in {config}")
            if key in settings:
                raise RosEnvironmentError(f"Duplicate ROS setting {key} in {config}")
            settings[key] = values[0]
        if settings.get("RMW_IMPLEMENTATION") != "rmw_cyclonedds_cpp":
            raise RosEnvironmentError(f"{config} must select rmw_cyclonedds_cpp")
        if not settings.get("CYCLONEDDS_URI"):
            raise RosEnvironmentError(f"{config} must include the services' CYCLONEDDS_URI")
        for key, value in settings.items():
            # ROS setup.bash exports a discovery-range default. Replace that
            # default with the services' policy, while rejecting overrides of
            # middleware, domain or the DDS network file.
            if (key in {"RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "CYCLONEDDS_URI"}
                    and key in environment and environment[key] != value):
                raise RosEnvironmentError(
                    f"{key} conflicts with robot services ({config}); unset {key} and retry")

    selected = dict(environment)
    selected.update(settings)
    selected.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    if selected["RMW_IMPLEMENTATION"] != "rmw_cyclonedds_cpp":
        raise RosEnvironmentError("Robot tools require rmw_cyclonedds_cpp; Fast DDS crashed on this Pi")
    try:
        domain = int(selected.get("ROS_DOMAIN_ID", "0"))
    except ValueError as error:
        raise RosEnvironmentError("ROS_DOMAIN_ID must be an integer from 0 to 232") from error
    if not 0 <= domain <= 232:
        raise RosEnvironmentError("ROS_DOMAIN_ID must be an integer from 0 to 232")
    uri = selected.get("CYCLONEDDS_URI")
    if uri:
        try:
            if uri.lstrip().startswith("<"):
                root = ET.fromstring(uri)
            else:
                root = ET.parse(Path(uri.removeprefix("file://")).expanduser()).getroot()
            if root.tag.split("}")[-1] != "CycloneDDS":
                raise ValueError("root must be CycloneDDS")
        except (OSError, ET.ParseError, ValueError) as error:
            raise RosEnvironmentError(f"Invalid CYCLONEDDS_URI {uri}: {error}") from error
    environment.update({key: value for key, value in selected.items() if key in ROS_ENV_KEYS})
    return {key: environment[key] for key in ROS_ENV_KEYS if key in environment}


def ensure_ros_environment():
    try:
        return configure_ros_environment()
    except (RosEnvironmentError, OSError) as error:
        print(f"ROS setup error: {error}; no ROS node started", file=sys.stderr)
        raise SystemExit(2) from None


if __name__ == "__main__":
    settings = ensure_ros_environment()
    if len(sys.argv) == 1:
        print(json.dumps(settings, sort_keys=True))
    else:
        os.execvpe(sys.argv[1], sys.argv[1:], os.environ)
