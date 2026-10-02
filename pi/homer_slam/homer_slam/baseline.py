"""Bounded JSON evidence capture; gates require comparison with a scan baseline."""
import argparse
import json
import math
from pathlib import Path
import statistics
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from .core import age_seconds


def cpu_sample():
    fields = list(map(int, Path('/proc/stat').read_text().splitlines()[0].split()[1:]))[:8]
    return sum(fields), fields[3]+fields[4]


class Baseline(Node):
    def __init__(self):
        super().__init__('homer_baseline')
        self.scans = 0; self.ages = []; self.samples = []; self.last_scan = self.last_odom = None
        self.odom_gaps = []; self.scan_gaps = []; self.last_scan_wall = self.last_odom_wall = None
        self.cpu = cpu_sample()
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.create_timer(1., self.sample)

    def scan(self, msg):
        now = time.monotonic()
        if self.last_scan_wall is not None: self.scan_gaps.append(now-self.last_scan_wall)
        self.last_scan_wall = now; self.last_scan = msg.header.stamp; self.scans += 1

    def odom(self, msg):
        now = time.monotonic()
        if self.last_odom_wall is not None: self.odom_gaps.append(now-self.last_odom_wall)
        self.last_odom_wall = now; self.last_odom = msg.header.stamp
        self.ages.append(age_seconds(self.get_clock().now().nanoseconds, msg.header.stamp))

    def sample(self):
        total,idle = cpu_sample(); previous_total,previous_idle = self.cpu; self.cpu = total,idle
        cpu = 100*(1-(idle-previous_idle)/max(1,total-previous_total))
        memory = {line.split(':')[0]: int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines() if line.startswith(('MemTotal:', 'MemAvailable:'))}
        thermal = Path('/sys/class/thermal/thermal_zone0/temp')
        temperature = int(thermal.read_text())/1000 if thermal.exists() else None
        try:
            throttling = subprocess.check_output(['vcgencmd','get_throttled'], text=True).strip()
        except (OSError, subprocess.CalledProcessError): throttling = None
        now = self.get_clock().now().nanoseconds
        self.samples.append({'time':time.time(), 'cpu_percent':cpu, 'memory_kib':memory, 'temperature_c':temperature, 'throttling':throttling,
            'scan_age':age_seconds(now,self.last_scan) if self.last_scan else None,
            'odom_age':age_seconds(now,self.last_odom) if self.last_odom else None})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path); parser.add_argument('--seconds', type=float, default=600)
    parser.add_argument('--baseline-scan-hz', type=float)
    args = parser.parse_args()
    if args.seconds <= 0: parser.error('seconds must be positive')
    if args.output.exists(): parser.error('Evidence output already exists')
    rclpy.init(); node = Baseline(); started = time.monotonic()
    try:
        while time.monotonic()-started < args.seconds and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        elapsed = time.monotonic()-started
        ages = sorted(node.ages)
        p95 = ages[min(len(ages)-1, math.ceil(len(ages)*.95)-1)] if ages else None
        cpu = statistics.mean(s['cpu_percent'] for s in node.samples) if node.samples else None
        rate = node.scans/elapsed
        throttle_known = bool(node.samples) and all(s['throttling'] is not None for s in node.samples)
        throttle_clear = throttle_known and all(int(s['throttling'].split('=')[1],16) & 0xf == 0 for s in node.samples)
        gates = {'odom_age': p95 is not None and p95 < .2 and min(ages) >= -.05,
                 'cpu': cpu is not None and cpu < 75,
                 'scan_rate': abs(rate/args.baseline_scan_hz-1) <= .1 if args.baseline_scan_hz else None,
                 'throttling': throttle_clear if throttle_known else None,
                 'stream_continuity': bool(node.odom_gaps) and max(node.odom_gaps) < .5 and bool(node.scan_gaps) and max(node.scan_gaps) < .5}
        result = {'seconds':elapsed, 'scan_hz':rate, 'odom_age_p95':p95, 'cpu_mean':cpu,
                  'gates':gates, 'accepted':all(v is True for v in gates.values()),
                  'note':'Freshness and continuity do not certify tracking accuracy; unknown gates are not passes.',
                  'samples':node.samples, 'odom_ages':ages}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as stream: json.dump(result,stream,indent=2)
        node.destroy_node(); rclpy.shutdown()
        print(json.dumps({k:v for k,v in result.items() if k not in ('samples','odom_ages')},indent=2))
