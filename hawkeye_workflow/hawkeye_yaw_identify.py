#!/usr/bin/env python3
"""Hawkeye Gazebo: bounded open-loop yaw-rate pulse test, preserving controllers.

Requires existing ROS 2 Hawkeye workflow, hold world, and no other /cam_vel
publishers. Uses existing homing and carrier-phase helpers without modifying them.
Saves raw command/feedback time histories; does NOT claim ZN gain estimates.
"""
import argparse
import csv
import datetime
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from npso_trial import ExperimentNode, wait_for_feedback
from repro_tools import reset, home_gimbal, arm, wait_orientation

DATA = Path.home() / 'hawkeye_x3_ws/rosbags/hawkeye_workflow'


def preflight():
    state_file = DATA / 'runtime.json'
    if not state_file.exists():
        raise RuntimeError('No managed stack. Run ./hawkeye start hold --seed 42')
    state = json.loads(state_file.read_text())
    if state.get('mode') != 'hold':
        raise RuntimeError('Expected hold world; current mode='+str(state.get('mode')))
    if (DATA / 'trial.lock').exists():
        raise RuntimeError('trial.lock exists: do not run concurrent experiments')
    result = subprocess.run(['ros2','topic','info','/cam_vel'],capture_output=True,text=True,timeout=10)
    if result.returncode:
        raise RuntimeError('No /cam_vel topic; check ./hawkeye status')
    p = re.search(r'Publisher count:\s*(\d+)',result.stdout)
    s = re.search(r'Subscription count:\s*(\d+)',result.stdout)
    if p is None or s is None or int(p.group(1)) != 0 or int(s.group(1)) < 1:
        raise RuntimeError('Expected /cam_vel pub=0 and sub>=1; got '+result.stdout)
    return state


class Probe(Node):
    def __init__(self):
        super().__init__('hawkeye_yaw_identification')
        self.publisher = self.create_publisher(Twist, '/cam_vel', 10)
        self.orientation = None
        self.last_feedback = None
        self.samples = []
        self.command = 0.0
        self.first_t = None
        self.start_ns = None
        self.create_subscription(Vector3, '/current_orientation', self.feedback, 10)
        self.create_timer(0.02, self.tick)

    def feedback(self, msg):
        now = time.monotonic()
        self.orientation = (float(msg.x),float(msg.y),float(msg.z))
        self.last_feedback = now
        if self.start_ns is not None:
            self.samples.append((time.monotonic_ns(), 'feedback', self.command,
                                 self.orientation[0], self.orientation[2]))

    def send(self, yaw):
        msg = Twist()
        msg.angular.y = float(yaw)  # physical yaw rate; angular.x = pitch = 0
        self.publisher.publish(msg)

    def tick(self):
        self.send(self.command)
        if self.start_ns is not None:
            pitch, yaw = ((float('nan'), float('nan')) if self.orientation is None
                          else (self.orientation[0], self.orientation[2]))
            self.samples.append((time.monotonic_ns(), 'command', self.command, pitch, yaw))


def run(args):
    preflight()
    if not (0 < args.rate <= 0.08 and .2 <= args.pulse <= 2.0):
        raise ValueError('Require 0<rate<=0.08 rad/s and 0.2<=pulse<=2 s')
    if args.rate * args.pulse > .12:
        raise ValueError('Pulse displacement must be <= 0.12 rad')
    folder = DATA / ('yaw_identification_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    folder.mkdir(parents=True,exist_ok=False)
    lock = DATA / 'trial.lock'
    lock.write_text(f'{os.getpid()} yaw_identification {folder}\n')
    rclpy.init()
    setup_node = None
    node = None
    meta = {'rate_rad_s':args.rate,'pulse_s':args.pulse,
            'segments_s':{'baseline':.8,'positive':args.pulse,'rest1':1.5,
                          'negative':args.pulse,'rest2':1.5},
            'world':'hold','success':False,'note':'open-loop yaw velocity input; not ZN tuning gains'}
    try:
        setup_node = ExperimentNode()
        wait_orientation(setup_node,20)
        setup_node.hold_zero(.5)
        carrier=reset()
        if carrier.get('mode')!='hold':
            raise RuntimeError('Carrier mode mismatch')
        meta['carrier_reset']=carrier
        home=home_gimbal(setup_node)
        meta['home']=home
        setup_node.hold_zero(.5)
        setup_node.destroy_node()
        setup_node=None
        node=Probe()
        deadline=time.monotonic()+10
        while node.last_feedback is None:
            if time.monotonic()>deadline: raise RuntimeError('No orientation feedback')
            rclpy.spin_once(node,timeout_sec=.05)
        if abs(node.orientation[2]) > 5 or abs(node.orientation[0]) > 5:
            raise RuntimeError('Gimbal not near home; initial orientation '+str(node.orientation))
        meta['initial_orientation_deg']={'pitch':node.orientation[0], 'yaw':node.orientation[2]}
        meta['carrier_armed']=arm(carrier['epoch'])

        # Carrier arming can block ROS callback processing.
        # Require a NEW orientation message after arming.
        node.last_feedback = None
        deadline = time.monotonic() + 5.0

        while node.last_feedback is None:
            if time.monotonic() > deadline:
                raise RuntimeError(
                    'No fresh orientation feedback after carrier arming'
                )
            rclpy.spin_once(node, timeout_sec=0.05)

        print('Fresh orientation feedback acquired after arming',
              flush=True)

        # Stable, time-based command schedule; 50 Hz publishing, feedback asynchronous.
        segments=[('baseline',.8,0.0),('positive',args.pulse,args.rate),
                  ('rest1',1.5,0.0),('negative',args.pulse,-args.rate),('rest2',1.5,0.0)]
        t0=time.monotonic()
        node.start_ns=time.monotonic_ns()
        schedule=[]
        acc=0.0
        for name,length,vel in segments:
            schedule.append((name,acc,acc+length,vel))
            acc+=length
        print('Recording',folder,flush=True)
        print('Home:',meta['initial_orientation_deg'],flush=True)
        print('Pulse schedule:',schedule,flush=True)
        current_segment=None
        while True:
            t=time.monotonic()-t0
            if t>=acc: break
            entry=next(s for s in schedule if s[1]<=t<s[2])
            if entry[0]!=current_segment:
                current_segment=entry[0]
                print(f'  {t:.3f}s {current_segment}: yaw rate {entry[3]:+.4f} rad/s',flush=True)
            node.command=entry[3]
            if node.last_feedback is None or time.monotonic()-node.last_feedback>0.5:
                raise RuntimeError('Stale yaw orientation feedback; stopping')
            rclpy.spin_once(node,timeout_sec=.005)
        node.command=0.0
        for _ in range(40):
            rclpy.spin_once(node,timeout_sec=.02)
            node.send(0.0)
        meta['success']=True
        meta['duration_s']=time.monotonic()-t0
        meta['final_orientation_deg']={'pitch':node.orientation[0],'yaw':node.orientation[2]}
    except BaseException as exc:
        meta['error']=repr(exc)
        raise
    finally:
        if node is not None:
            node.command=0.0
            for _ in range(30):
                try:
                    node.send(0.0)
                    rclpy.spin_once(node,timeout_sec=.02)
                except Exception:
                    pass
            if node.start_ns is not None:
                with (folder/'yaw_response.csv').open('w',newline='') as f:
                    w=csv.writer(f)
                    w.writerow(['time_s','source','command_yaw_rad_s','pitch_deg','yaw_deg'])
                    w.writerows([(round((t-node.start_ns)*1e-9,9),source,command,pitch,yaw)
                                 for t,source,command,pitch,yaw in node.samples])
                meta['row_count']=len(node.samples)
            node.destroy_node()
        if setup_node is not None:
            try: setup_node.hold_zero(.5)
            except Exception: pass
            setup_node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        (folder/'run.json').write_text(json.dumps(meta,indent=2,default=str)+'\n')
        lock.unlink(missing_ok=True)
        print('Saved',folder/'yaw_response.csv',flush=True)
        print('Result:', 'PASS' if meta['success'] else 'FAILED',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rate',type=float,default=.05)
    p.add_argument('--pulse',type=float,default=1.0)
    a=p.parse_args()
    try: run(a)
    except Exception as exc:
        print('ERROR:',exc,file=sys.stderr)
        sys.exit(1)
