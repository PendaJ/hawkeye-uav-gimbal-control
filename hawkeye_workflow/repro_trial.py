#!/usr/bin/env python3
"""A single deterministic-initialization / carrier-phase-gated Hawkeye trial.

Bags and run.json are retained even when an evaluation is invalid. The exact
control period is parsed from controller logging; off mode uses wall-clock.
The carrier trigger occurs on the FIRST nonzero joint command after arm.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import rclpy
from npso_trial import ExperimentNode, wait_for_feedback, prepare_pose
from repro_tools import reset, home_gimbal, arm, manual_start, state, wait_orientation


def sha256(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def stop(proc,wait=15):
    if proc is None or proc.poll() is not None:return
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=wait)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid,signal.SIGTERM)
        try:proc.wait(timeout=4)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL)


def run(args):
    args.path.mkdir(parents=True,exist_ok=False)
    config={
        'controller':args.controller,'world':args.world,'duration_s':args.duration,
        'seed':args.seed,'initial_offset_px':[args.initial_x,args.initial_y],
        'initial_offset_tolerance_px':args.tolerance,
        'repro_method':'carrier reset + gimbal home + matched image error + phase trigger',
        'carrier_trajectory':'y=0.35*sin(0.5*t) from first command; x=1,z=1.2 (combined adds yaw +/-5 and pitch +/-4 deg)',
        'gains':None if args.controller=='off' else {'kp':args.kp,'kd':args.kd,'omega_o':args.wo},
        'controller_sha256':sha256(args.controller_file),
        'plugin_sha256':sha256(args.plugin),
        'bag':str(args.path/'bag'),
        'run_directory':str(args.path),
        'success':False,
    }
    rclpy.init()
    node=ExperimentNode()
    bagproc=None
    controller_proc=None
    cmdlog=None
    blog=None
    try:
        wait_orientation(node,20)
        node.hold_zero(.4)
        r=reset()
        if r['mode'] != args.world:raise RuntimeError('Carrier world mode mismatch')
        config['carrier_reset_state']=r
        config['initial_home_deg']=home_gimbal(node)
        target,angles=prepare_pose(node,args.initial_x,args.initial_y,args.tolerance,50)
        config['initial_target_px']=list(target)
        config['initial_gimbal_deg']={'pitch':angles[0],'yaw':angles[1]}
        print('HOMED + MATCHED:',config['initial_gimbal_deg'],config['initial_target_px'],flush=True)
        bag_cmd=['ros2','bag','record','-o',str(args.path/'bag')]
        if args.qos.exists():bag_cmd+=['--qos-profile-overrides-path',str(args.qos)]
        bag_cmd+=['/target_coord','/current_orientation','/cam_vel','/pid_log',
                  '/hawkeye/carrier/state']
        blog=open(args.path/'bag.log','w')
        bagproc=subprocess.Popen(bag_cmd,stdout=blog,stderr=subprocess.STDOUT,start_new_session=True)
        for _ in range(30):
            node.spin(.1)
            if bagproc.poll() is not None:raise RuntimeError('Bag recorder exited early')
        node.hold_zero(.5)
        target_now,angles_now=node.feedback(max_age=.9)
        if abs(target_now[0]-args.initial_x)>args.tolerance or abs(target_now[1]-args.initial_y)>args.tolerance:
            raise RuntimeError('Initial target offset changed before control: '+str(target_now))
        config['pretrial_target_px']=list(target_now)
        config['carrier_armed_state']=arm(r['epoch'])
        print('ARMED: carrier remains at y=0 until the first nonzero control command',flush=True)
        if args.controller=='off':
            config['controller_process_start_ns']=time.time_ns()
            config['carrier_started_state']=manual_start(r['epoch'])
            print(f'OPEN LOOP: {args.duration:.1f} seconds',flush=True)
            time.sleep(args.duration)
            config['controller_process_end_ns']=time.time_ns()
            rc=0
        else:
            cmd=[args.python,str(args.controller_file),'--ros-args',
                 '-p','enable_motion:=true','-p',f'run_seconds:={args.duration}',
                 '-p',f'kp_yaw:={args.kp}','-p',f'kd_yaw:={args.kd}',
                 '-p',f'omega_o_yaw:={args.wo}',
                 '-p',f'kp_pitch:={args.kp}','-p',f'kd_pitch:={args.kd}',
                 '-p',f'omega_o_pitch:={args.wo}']
            cmdlog=open(args.path/'controller.log','w')
            config['controller_process_start_ns']=time.time_ns()
            controller_proc=subprocess.Popen(cmd,stdout=cmdlog,stderr=subprocess.STDOUT,
                                             start_new_session=True)
            try:rc=controller_proc.wait(timeout=args.duration+35)
            except subprocess.TimeoutExpired:
                stop(controller_proc,4)
                raise RuntimeError('Controller watchdog timeout')
            config['controller_process_end_ns']=time.time_ns()
            cmdlog.flush()
            text=(args.path/'controller.log').read_text(errors='replace')
            m=re.search(r'Control duration\s+([\d.]+)\s+s completed',text)
            if rc!=0 or not m or abs(float(m.group(1))-args.duration)>.2:
                raise RuntimeError('Controller did not complete timed interval; inspect controller.log')
            # Preserve actual timed window start for exact analysis.
            for line in text.splitlines():
                if 'Fresh target and feedback acquired. Timed control window started:' in line:
                    ts=re.search(r'\[(\d{10}(?:\.\d+)?)\]',line)
                    if ts:config['control_window_start_ns']=int(float(ts.group(1))*1e9)
                    break
            if 'control_window_start_ns' in config:
                config['control_window_end_ns']=config['control_window_start_ns']+int(args.duration*1e9)
        config['carrier_after_state']=state()
        if not config['carrier_after_state'].get('active') or config['carrier_after_state']['epoch'] != r['epoch']:
            raise RuntimeError('Carrier never started at commanded phase')
        config['controller_returncode']=rc
        config['success']=True
        print('TRIAL PASS: elapsed_carrier_sim_s=',config['carrier_after_state']['elapsed'],flush=True)
    except BaseException as exc:
        config['error']=repr(exc)
        print('TRIAL FAILED:',exc,file=sys.stderr,flush=True)
        raise
    finally:
        stop(controller_proc,4)
        try:node.hold_zero(.6)
        except Exception:pass
        stop(bagproc,20)
        if cmdlog:cmdlog.close()
        if blog:blog.close()
        node.destroy_node()
        rclpy.shutdown()
        (args.path/'run.json').write_text(json.dumps(config,indent=2)+'\n')


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--path',type=Path,required=True)
    ap.add_argument('--world',choices=['hold','lateral','combined'],required=True)
    ap.add_argument('--controller',choices=['off','sspd','zn_sspd','zn_pd','pd_custom','custom'],required=True)
    ap.add_argument('--duration',type=float,default=60)
    ap.add_argument('--initial-x',type=float,default=80)
    ap.add_argument('--initial-y',type=float,default=28)
    ap.add_argument('--tolerance',type=float,default=5)
    ap.add_argument('--seed',type=int,default=42)
    ap.add_argument('--kp',type=float,default=9.6348)
    ap.add_argument('--kd',type=float,default=.1508)
    ap.add_argument('--wo',type=float,default=600.2994)
    ap.add_argument('--python',required=True)
    ap.add_argument('--controller-file',type=Path,required=True)
    ap.add_argument('--plugin',type=Path,required=True)
    ap.add_argument('--qos',type=Path,required=True)
    a=ap.parse_args()
    try:run(a)
    except Exception:sys.exit(1)

if __name__=='__main__':main()
