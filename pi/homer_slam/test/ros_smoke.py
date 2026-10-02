#!/usr/bin/env python3
"""Isolated-domain stationary synthetic scan, save/reload and stale-data test.
Run with Jazzy and the Homer workspace sourced. Never publishes drive commands.
"""
import math
import random
import argparse
import os
import signal
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import MarkerArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from tf2_ros import Buffer, TransformListener
from rosgraph_msgs.msg import Clock


class Probe(Node):
    def __init__(self, sim=False):
        super().__init__('homer_synthetic_probe', parameter_overrides=[Parameter('use_sim_time', value=sim)])
        self.active = True; self.invalid_scan = False; self.odom = None; self.map = None; self.markers = None; self.diagnostics = None; self.viewer = None
        self.buffer = Buffer(node=self); self.listener = TransformListener(self.buffer,self)
        self.pub = self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',lambda m:setattr(self,'odom',m),10)
        self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'map',m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(MarkerArray,'/homer/robot_markers',lambda m:setattr(self,'markers',m),10)
        self.create_subscription(DiagnosticArray,'/diagnostics',lambda m:setattr(self,'diagnostics',m),10)
        self.create_subscription(MarkerArray,'/homer/view_status',lambda m:setattr(self,'viewer',m),10)
        self.create_timer(.1,self.scan)

    def scan(self):
        if not self.active:return
        msg=LaserScan(); msg.header.frame_id='laser'; msg.header.stamp=self.get_clock().now().to_msg()
        msg.angle_min=-math.pi; msg.angle_increment=2*math.pi/720; msg.angle_max=msg.angle_min+719*msg.angle_increment
        msg.range_min=.15; msg.range_max=8.; msg.scan_time=.1; msg.time_increment=0.
        msg.ranges=[]
        for index in range(720):
            a=msg.angle_min+index*msg.angle_increment
            c,s=math.cos(a),math.sin(a)
            # Identical noiseless scans make RF2O's robust covariance singular.
            msg.ranges.append(min(3/max(abs(c),1e-6),2/max(abs(s),1e-6))+random.gauss(0,.001))
        if self.invalid_scan: msg.ranges = [float('inf')] * len(msg.ranges)
        self.pub.publish(msg)


def wait_for(predicate, label, seconds=30):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.1)
    raise AssertionError('Timed out: '+label)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--sim-time',action='store_true');args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '71','Run with ROS_DOMAIN_ID=71 to isolate real robot nodes'
    random.seed(42)
    rclpy.init(); probe=Probe(args.sim_time); executor=SingleThreadedExecutor(); executor.add_node(probe)
    clock_active=threading.Event();clock_active.set()
    clock_pub=probe.create_publisher(Clock,'/clock',10)
    def clock():
        started=time.monotonic()
        while clock_active.is_set():
            stamp=int((100+(time.monotonic()-started)*.5)*1e9)
            msg=Clock();msg.clock.sec=stamp//1_000_000_000;msg.clock.nanosec=stamp%1_000_000_000
            clock_pub.publish(msg);time.sleep(.02)
    clock_thread=threading.Thread(target=clock,daemon=True) if args.sim_time else None
    if clock_thread:clock_thread.start()
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    process=None;viewer=None
    with tempfile.TemporaryDirectory(prefix='homer-smoke-') as directory:
        log=open(Path(directory)/'mapping.log','w+')
        try:
            base=['ros2','launch','homer_slam']
            arguments=['min_range:=0.15','max_range:=8.0']
            if args.sim_time:arguments += ['use_sim_time:=true']
            viewer=subprocess.Popen(['ros2','run','homer_slam','viewer_status','--ros-args','-p','use_sim_time:='+str(args.sim_time).lower()],stdout=log,stderr=subprocess.STDOUT)
            invalid=subprocess.run(base+['odometry.launch.py','geometry:=/nonexistent/geometry.yaml']+arguments,capture_output=True,text=True,timeout=20)
            assert invalid.returncode!=0 and 'geometry.yaml' in invalid.stdout+invalid.stderr
            invalid=subprocess.run(base+['localization.launch.py','map_file:=/nonexistent/map']+arguments,capture_output=True,text=True,timeout=20)
            assert invalid.returncode!=0 and 'map' in invalid.stdout+invalid.stderr
            empty_pub=probe.create_publisher(OccupancyGrid, '/map', QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            empty_pub.publish(OccupancyGrid())
            empty_save=subprocess.run(['ros2','run','homer_slam','save_map',str(Path(directory)/'empty-save'),'--timeout','5'],capture_output=True,text=True,timeout=15)
            assert empty_save.returncode != 0 and 'empty' in empty_save.stdout+empty_save.stderr
            probe.destroy_publisher(empty_pub); probe.map=None
            probe.invalid_scan = True
            process=subprocess.Popen(base+['mapping.launch.py']+arguments,stdout=log,stderr=subprocess.STDOUT)
            wait_for(lambda:probe.diagnostics and any('insufficient valid laser ranges' in status.message for status in probe.diagnostics.status),'initial invalid scans',30)
            assert probe.odom is None, 'Invalid startup scans must not initialize odometry'
            probe.invalid_scan = False
            wait_for(lambda:probe.odom and probe.map and probe.map.info.width > 0 and probe.markers,'odometry, map and markers',60)
            assert probe.map.info.width > 0 and probe.map.info.height > 0 and any(v >= 0 for v in probe.map.data)
            q=probe.odom.pose.pose.orientation
            assert all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w,probe.odom.pose.pose.position.x))
            assert abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<.01
            assert probe.odom.header.frame_id=='odom' and probe.odom.child_frame_id=='base_link'
            if args.sim_time:assert 100<=probe.odom.header.stamp.sec<200
            assert probe.markers.markers[0].header.frame_id=='base_link'
            assert probe.markers.markers[0].lifetime.sec==0
            wait_for(lambda:probe.buffer.can_transform('map','laser',rclpy.time.Time()),'complete TF')
            mount=probe.buffer.lookup_transform('base_link','laser',rclpy.time.Time()).transform.translation
            assert abs(mount.x-.09)<1e-6 and abs(mount.y-.075)<1e-6 and abs(mount.z-.093)<1e-6
            outline=probe.markers.markers[1].points
            assert abs(outline[0].x+.159)<1e-6 and abs(outline[0].y+.175)<1e-6
            wait_for(lambda:probe.diagnostics and all(s.level==DiagnosticStatus.OK for s in probe.diagnostics.status),'healthy diagnostics',15)
            wait_for(lambda:probe.viewer and probe.viewer.markers[0].action==2,'viewer clears fresh-data warning',15)
            duplicate=subprocess.run(base+['odometry.launch.py']+arguments,capture_output=True,text=True,timeout=20)
            assert duplicate.returncode!=0 and 'Another Homer' in duplicate.stdout+duplicate.stderr
            probe.invalid_scan = True
            wait_for(lambda: probe.diagnostics and any('insufficient valid laser ranges' in status.message for status in probe.diagnostics.status), 'invalid ranges diagnosed', 15)
            time.sleep(1.)
            last_stamp = probe.odom.header.stamp
            time.sleep(1.)
            assert probe.odom.header.stamp == last_stamp, 'Invalid scans must not refresh odometry'
            probe.invalid_scan = False
            wait_for(lambda: probe.diagnostics and all(s.level == DiagnosticStatus.OK for s in probe.diagnostics.status), 'recovery after invalid scans', 15)
            saved=Path(directory)/'saved'
            subprocess.run(['ros2','run','homer_slam','save_map',str(saved)],check=True,timeout=100)
            duplicate_save=subprocess.run(['ros2','run','homer_slam','save_map',str(saved)],capture_output=True,timeout=15)
            assert duplicate_save.returncode!=0
            probe.active=False
            wait_for(lambda:probe.diagnostics and any(s.message.startswith('stale') for s in probe.diagnostics.status),'stale diagnostics',10)
            wait_for(lambda:probe.viewer and 'stale' in probe.viewer.markers[0].text,'local viewer stale warning',10)
            process.send_signal(signal.SIGINT); process.wait(timeout=20);process=None
            failed=Path(directory)/'failed-save'
            failure=subprocess.run(['ros2','run','homer_slam','save_map',str(failed),'--timeout','1'],capture_output=True,text=True,timeout=15)
            assert failure.returncode!=0 and not failed.exists()
            assert 'Partial exports retained' in failure.stdout
            probe.map=None;probe.odom=None;probe.active=True
            process=subprocess.Popen(base+['localization.launch.py', 'map_file:='+str(saved/'map')]+arguments,stdout=log,stderr=subprocess.STDOUT)
            wait_for(lambda:probe.map and probe.odom,'localization restart',60)
            print('PASS: synthetic scan -> odometry/TF/map/markers; save/refuse overwrite; stale data; localization reload')
        except Exception:
            if process and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=20)
                except subprocess.TimeoutExpired: process.kill(); process.wait()
            log.flush();log.seek(0);print(log.read()[-16000:]);raise
        finally:
            if viewer:
                viewer.send_signal(signal.SIGINT)
                try:viewer.wait(timeout=10)
                except subprocess.TimeoutExpired:viewer.kill();viewer.wait()
            if process:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=20)
                except subprocess.TimeoutExpired:process.kill();process.wait()
            clock_active.clear()
            if clock_thread:clock_thread.join(timeout=3)
            executor.shutdown();thread.join(timeout=3);probe.destroy_node();rclpy.shutdown();log.close()


if __name__=='__main__':main()
