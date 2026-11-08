#!/usr/bin/env python3
"""Offline NPSO tuning of Hawkeye SS-PD with real Gazebo/ROS 2 closed-loop trials.

Paper-aligned: eta=[Kp,Kd,omega_o]; 20 particles x 11 evaluation rounds,
10 update rounds; global-best-relative position and RETAINED particle velocities;
positive 90th-percentile rational metric normalizers fixed across optimizations.

Live mode uses npso_trial.py (adapted from the user's proven repeatability runner),
existing safe controller, ROS 2 bag QoS configuration, and YOLO tracker.
"""
from __future__ import annotations
import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import sys
import time
from typing import Any
import numpy as np

LOW = np.array([7., 0., 20.], dtype=float)
HIGH = np.array([14., 0.70, 1000.], dtype=float)
VMAX = np.array([0.7, 0.07, 100.], dtype=float)
METRICS = ('mae', 'volatility', 'acceleration', 'zero_crossings')
DEFAULT_WEIGHTS = (1., 1., 1., 1.)  # Exp. 6


def log(msg: str):
    print(msg, flush=True)


def safe_json(path: Path, payload: Any):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + '\n')
    os.replace(tmp, path)


def metrics_from_errors(ex: np.ndarray, ey: np.ndarray) -> dict:
    if len(ex) < 3 or len(ex) != len(ey):
        raise ValueError('Need at least three paired measurements')
    ep = np.hypot(ex, ey)
    delta = np.diff(ep)
    d2 = np.diff(ep, n=2)
    def signs(a):
        s = np.sign(a).astype(float)
        last = 1.0
        for i in range(len(s)):
            if s[i] == 0:
                s[i] = last
            else:
                last = s[i]
        return s
    zc = int(np.count_nonzero(np.diff(signs(ex))) + np.count_nonzero(np.diff(signs(ey))))
    return dict(mae=float(np.mean(ep)), volatility=float(np.mean(np.abs(delta))),
                acceleration=float(np.mean(np.abs(d2))), zero_crossings=float(zc),
                samples=int(len(ep)), initial_radial_px=float(ep[0]),
                final_radial_px=float(ep[-1]))


def scalar_fitness(raw: dict, norm: dict, weights: tuple[float, ...]):
    bounded = {key: float(raw[key]) / (float(raw[key]) + float(norm[key])) for key in METRICS}
    return float(sum(w * bounded[m] for w, m in zip(weights, METRICS))), bounded


def controller_start_ns(log_text: str) -> int:
    # ROS 2 controller logger timestamps are UNIX wall-clock seconds in this setup.
    for line in log_text.splitlines():
        if 'Fresh target and feedback acquired. Timed control window started:' in line:
            m = re.search(r'\[(\d{10}(?:\.\d+)?)\]', line)
            if m:
                return int(float(m.group(1)) * 1e9)
    raise RuntimeError('Controller startup timestamp missing in controller.log; cannot align fitness window safely')


def bag_tracking(bag_dir: Path, start_ns: int, duration: float) -> tuple[np.ndarray,np.ndarray,np.ndarray,np.ndarray]:
    """Decode actual /target_coord ROS 2 Point samples during active control."""
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import Point
    stop_ns = start_ns + int(duration * 1e9)
    records = []
    for db in sorted(bag_dir.glob('*.db3')):
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        try:
            rows = con.execute('''SELECT m.timestamp,m.data FROM messages m JOIN topics t
                                  ON m.topic_id=t.id WHERE t.name='/target_coord'
                                  AND m.timestamp BETWEEN ? AND ? ORDER BY m.timestamp''',
                               (int(start_ns),int(stop_ns)))
            for stamp, blob in rows:
                msg = deserialize_message(blob, Point)
                records.append((int(stamp),float(msg.x),float(msg.y),float(msg.z)))
        finally:
            con.close()
    records.sort(key=lambda r:r[0])
    if len(records) < 15:
        raise RuntimeError(f'Only {len(records)} in-control target samples (minimum 15)')
    a = np.asarray(records, dtype=float)
    if not np.all(np.isfinite(a)) or np.any(a[:,3] < 0.5):
        raise RuntimeError('Lost target or nonfinite target measurements during evaluation')
    # Account for the leading and trailing interval when checking sample gaps.
    timestamps = np.r_[start_ns, a[:,0], stop_ns].astype(np.float64)
    gap = float(np.max(np.diff(timestamps))/1e9)
    if gap > 0.75:
        raise RuntimeError(f'Maximum in-control vision gap {gap:.3f} s exceeded 0.75 s')
    return a[:,0],a[:,1],a[:,2],a[:,3]


@dataclass
class Particle:
    x: np.ndarray
    v: np.ndarray
    pbest: np.ndarray
    pbest_f: float = math.inf

    def as_json(self):
        return {'x':self.x.tolist(), 'v':self.v.tolist(), 'pbest':self.pbest.tolist(),
                'pbest_f':self.pbest_f if math.isfinite(self.pbest_f) else None}

    @classmethod
    def from_json(cls, v):
        return cls(np.array(v['x'],float),np.array(v['v'],float),np.array(v['pbest'],float),
                   float(v['pbest_f']) if v['pbest_f'] is not None else math.inf)


class NPSO:
    """Online sequential best updates; 1 initial round + n_rounds-1 update rounds."""
    def __init__(self, seed:int, n_particles:int, n_rounds:int):
        self.seed=seed
        self.n_particles=n_particles
        self.n_rounds=n_rounds
        self.rng=np.random.default_rng(seed)
        self.particles=[]
        for _ in range(n_particles):
            x=self.rng.uniform(LOW,HIGH)
            v=self.rng.uniform(-VMAX,VMAX)
            self.particles.append(Particle(x,v,x.copy()))
        self.best_x=None
        self.best_f=math.inf
        self.next_eval=0
        self.evaluations=[]

    @property
    def budget(self):
        return self.n_particles * self.n_rounds

    def proposal(self):
        idx=self.next_eval
        round_idx,particle_idx=divmod(idx,self.n_particles)
        p=self.particles[particle_idx]
        if round_idx>0:
            if self.best_x is None:
                raise RuntimeError('No global best after initial population')
            k=round_idx-1
            rho=k/self.n_rounds
            w=0.9-0.5*rho
            c1=2.0*(1.0-rho**1.5)
            c2=2.0*(rho**1.5)
            # Inertia, cognition, social terms retain velocity history.
            r1=self.rng.random(3)
            r2=self.rng.random(3)
            p.v=np.clip(w*p.v+c1*r1*(p.pbest-p.x)+c2*r2*(self.best_x-p.x),-VMAX,VMAX)
            # Key NPSO distinction: update position relative to CURRENT global best.
            p.x=np.clip(self.best_x+p.v,LOW,HIGH)
        return round_idx,particle_idx,p.x.copy()

    def accept(self,fitness:float,metadata:dict):
        if not math.isfinite(fitness) or fitness<0:
            raise ValueError('Fitness must be finite and nonnegative')
        rid,pid=divmod(self.next_eval,self.n_particles)
        p=self.particles[pid]
        if fitness<p.pbest_f:
            p.pbest_f=fitness
            p.pbest=p.x.copy()
        if fitness<self.best_f:
            self.best_f=fitness
            self.best_x=p.x.copy()
        self.evaluations.append({'eval':self.next_eval+1,'round':rid,'particle':pid+1,
                                 'kp':float(p.x[0]),'kd':float(p.x[1]),
                                 'omega_o':float(p.x[2]),'fitness':fitness,
                                 'global_best_fitness':self.best_f,**metadata})
        self.next_eval+=1

    def state(self):
        return {'seed':self.seed,'particles_n':self.n_particles,'rounds_n':self.n_rounds,
                'next_eval':self.next_eval,'best_f':self.best_f if math.isfinite(self.best_f) else None,
                'best_x':self.best_x.tolist() if self.best_x is not None else None,
                'particles':[p.as_json() for p in self.particles],
                'rng_state':self.rng.bit_generator.state,'evaluations':self.evaluations}

    @classmethod
    def restore(cls,s):
        o=cls(s['seed'],s['particles_n'],s['rounds_n'])
        o.next_eval=s['next_eval']
        o.best_f=float(s['best_f']) if s['best_f'] is not None else math.inf
        o.best_x=np.array(s['best_x'],float) if s['best_x'] is not None else None
        o.particles=[Particle.from_json(p) for p in s['particles']]
        o.rng.bit_generator.state=s['rng_state']
        o.evaluations=s['evaluations']
        return o


def export_csv(out: Path,evaluations: list):
    if not evaluations:
        return
    keys=list(dict.fromkeys(k for row in evaluations for k in row))
    with out.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys)
        w.writeheader()
        w.writerows(evaluations)


def prepare_ros(args):
    # Lazy import: allows deterministic NPSO unit tests on machines without ROS 2.
    import rclpy
    from npso_trial import ExperimentNode,wait_for_feedback,run_trial
    rclpy.init()
    node=ExperimentNode()
    try:
        if args.repro:
            from repro_tools import wait_orientation
            wait_orientation(node,timeout=15.0)
        else:
            wait_for_feedback(node, max_seconds=15.0)
    except BaseException:
        node.destroy_node()
        rclpy.shutdown()
        raise
    return rclpy,node,run_trial


def trial_args(args,x,output_session):
    from argparse import Namespace
    return Namespace(kp=float(x[0]),kd=float(x[1]),omega_o=float(x[2]),
                     target_x=args.target_x,target_y=args.target_y,
                     tolerance=args.tolerance,position_timeout=args.position_timeout,
                     duration=args.duration,max_vision_gap=0.75,
                     controller=args.controller,qos=args.qos,
                     runs=1,output=output_session,
                     repro=args.repro, carrier_mode=args.world_mode, home_timeout=35.0)


def evaluate_live(node,run_trial,args,x,out_dir:Path,trial_number:int):
    """Each candidate is a NEW matched-error closed-loop trial; never replay old bags."""
    targs=trial_args(args,x,out_dir)
    # On resume, do not overwrite partially recorded attempts from a crash.
    existing=[int(p.name.rsplit('_',1)[-1]) for p in out_dir.glob(f'run_{trial_number:02d}_attempt_*')
              if p.is_dir() and p.name.rsplit('_',1)[-1].isdigit()]
    first_attempt=max(existing,default=0)+1
    for attempt in range(first_attempt,first_attempt+args.max_retries+1):
        meta=run_trial(node,targs,out_dir,trial_index=trial_number,attempt=attempt)
        if not meta.get('success'):
            log(f'  candidate attempt {attempt} rejected: {meta.get("error","unknown")}')
            continue
        d=out_dir/f'run_{trial_number:02d}_attempt_{attempt:02d}'
        start_ns=controller_start_ns((d/'controller.log').read_text())
        try:
            times,ex,ey,det=bag_tracking(d/'bag',start_ns,args.duration)
            raw=metrics_from_errors(ex,ey)
        except RuntimeError as exc:
            log(f'  candidate attempt {attempt} rejected during scoring: {exc}')
            continue
        raw.update({'max_vision_gap_s':meta['max_recorded_vision_gap_during_controller_s'],
                    'detected_fraction':float(np.mean(det>=0.5))})
        return raw,str(d),meta
    raise RuntimeError('Candidate could not be evaluated reliably after retries. '
                       'Optimization is paused; fix Gazebo/YOLO load then resume.')


def calibrate(args):
    """Separate pilot evaluations establish fixed P90 references for all later seeds."""
    rng=np.random.default_rng(args.seed)
    out=args.output/('npso_pilot_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True,exist_ok=False)
    rclpy,node,run_trial=prepare_ros(args)
    rows=[]
    try:
        for i in range(args.pilot_evals):
            x=rng.uniform(LOW,HIGH)
            log(f'PILOT {i+1}/{args.pilot_evals} gains {x}')
            raw,folder,meta=evaluate_live(node,run_trial,args,x,out,i+1)
            row={'eval':i+1,'seed':args.seed,'kp':float(x[0]),'kd':float(x[1]),
                 'omega_o':float(x[2]),**raw,'trial_folder':folder}
            rows.append(row)
            safe_json(out/'pilot_progress.json',{'evaluations':rows})
    finally:
        node.hold_zero()
        node.destroy_node()
        rclpy.shutdown()
    normalizers={m:float(np.percentile([r[m] for r in rows],90)) for m in METRICS}
    # If pilot zero crossing counts are all zero, use 1 crossing as positive ref.
    normalizers={m:max(1.0 if m=='zero_crossings' else 1e-6,c)
                 for m,c in normalizers.items()}
    data={'reference_constants':normalizers,'method':'P90 of pilot raw metrics; rational J/(J+c)',
          'pilot_n':len(rows),'pilot_seed':args.seed,'pilot_folder':str(out),
          'duration_s':args.duration,'initial_error_px':[args.target_x,args.target_y],
          'model':'Gazebo X3 + YOLO26n + ByteTrack + Hawkeye SS-PD (device recorded in tracker.log)',
          'world_mode':args.world_mode,'repro':args.repro,
          'metric_volatility':'mean absolute radial first difference (per valid vision sample)'}
    safe_json(args.normalizers,data)
    export_csv(out/'pilot_metrics.csv',rows)
    log(f'Calibration saved: {args.normalizers}\n{normalizers}')


def optimize(args):
    if not args.normalizers.is_file():
        raise FileNotFoundError('Run --mode pilot first to create normalizers.json.')
    info=json.loads(args.normalizers.read_text())
    normalizers=info['reference_constants']
    if (info.get('duration_s')!=args.duration or
        info.get('initial_error_px')!=[args.target_x,args.target_y] or
        info.get('world_mode')!=args.world_mode or info.get('repro')!=args.repro):
        raise ValueError('Pilot normalizers were calibrated for different initial errors or duration')
    for m in METRICS:
        if not math.isfinite(float(normalizers[m])) or normalizers[m]<=0:
            raise ValueError(f'Invalid normalizer {m}')
    if args.resume:
        out=args.resume.resolve()
        saved=json.loads((out/'checkpoint.json').read_text())
        if saved['weights']!=list(args.weights) or saved['normalizers']!=normalizers or \
           saved.get('world_mode')!=args.world_mode:
            raise ValueError('Weights or normalizers differ from checkpoint')
        swarm=NPSO.restore(saved['swarm'])
        if swarm.seed!=args.seed or swarm.n_particles!=args.particles or swarm.n_rounds!=args.rounds:
            raise ValueError('Seed or particle/round counts differ from checkpoint')
    else:
        out=args.output/('npso_seed_'+str(args.seed)+'_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
        out.mkdir(parents=True,exist_ok=False)
        swarm=NPSO(args.seed,args.particles,args.rounds)
        safe_json(out/'configuration.json',{'seed':args.seed,'particles':args.particles,
                  'rounds':args.rounds,'evaluations_budget':swarm.budget,
                  'weights':args.weights,'normalizers':normalizers,
                  'pilot':str(args.normalizers),'target_px':[args.target_x,args.target_y],
                  'duration_s':args.duration,'world_mode':args.world_mode,
                  'repro':args.repro,'algorithm':'global-best-relative NPSO; persistent velocity; immediate gbest update'})
    log(f'Session {out}; {swarm.next_eval}/{swarm.budget} completed; seed {swarm.seed}')
    rclpy,node,run_trial=prepare_ros(args)
    try:
        while swarm.next_eval<swarm.budget:
            rnd,pid,x=swarm.proposal()
            count=swarm.next_eval+1
            log(f'\nEvaluation {count}/{swarm.budget} round {rnd+1}/{swarm.n_rounds}, '
                f'particle {pid+1}/{swarm.n_particles}: Kp={x[0]:.5f}, '
                f'Kd={x[1]:.5f}, omega_o={x[2]:.3f}')
            raw,folder,_=evaluate_live(node,run_trial,args,x,out,count)
            fitness,bounded=scalar_fitness(raw,normalizers,tuple(args.weights))
            swarm.accept(fitness,{'raw_metrics':raw,'normalized_metrics':bounded,'trial_folder':folder})
            safe_json(out/'checkpoint.json',{'swarm':swarm.state(),'weights':args.weights,
                                            'normalizers':normalizers,'world_mode':args.world_mode})
            export_csv(out/'evaluations.csv',swarm.evaluations)
            log(f'  J={fitness:.5f}; global-best J={swarm.best_f:.5f}, '
                f'gbest={swarm.best_x.round(5)}')
    finally:
        node.hold_zero()
        node.destroy_node()
        rclpy.shutdown()
    best={'fitness':swarm.best_f,'kp':float(swarm.best_x[0]),
          'kd':float(swarm.best_x[1]),'omega_o':float(swarm.best_x[2]),
          'seed':args.seed,'evaluations':swarm.next_eval,'output_directory':str(out)}
    safe_json(out/'best_parameters.json',best)
    log('OPTIMIZATION COMPLETE: '+json.dumps(best))


def selftest():
    # Confirm determinism, boundaries, evaluation accounting and rational fitness.
    def f(x):
        return float(((x[0]-9.4)/4)**2+((x[1]-.15)/.7)**2+((x[2]-600)/1000)**2)
    traces=[]
    for _ in range(2):
        o=NPSO(42,4,3)
        values=[]
        while o.next_eval<o.budget:
            k,i,x=o.proposal()
            assert np.all(x>=LOW) and np.all(x<=HIGH)
            score=f(x)
            o.accept(score,{})
            values.append((tuple(x),score))
        assert len(values)==12
        assert all(o.evaluations[i]['global_best_fitness']>=o.evaluations[i+1]['global_best_fitness'] for i in range(11))
        traces.append(values)
    assert traces[0]==traces[1]
    assert NPSO(42,20,11).budget==220
    raw=metrics_from_errors(np.array([3.,-1.,1.,-3.]),np.array([0.,0.,0.,0.]))
    val,components=scalar_fitness(raw,{m:1. for m in METRICS},DEFAULT_WEIGHTS)
    assert 0<=val<4 and all(0<=v<1 for v in components.values())
    log('SELFTEST PASS: seeded runs identical; 4 x 3 = 12 synthetic evaluations; publication budget 20 x 11 = 220; bounds, checkpoint and normalization valid.')


def main():
    ap=argparse.ArgumentParser(description='Manuscript-aligned NPSO with Gazebo closed-loop fitness evaluations')
    ap.add_argument('--mode',choices=['selftest','smoke','pilot','optimize'],default='selftest')
    ap.add_argument('--seed',type=int,default=42)
    ap.add_argument('--world-mode',choices=['hold','lateral','combined'],default='lateral')
    ap.add_argument('--repro',action='store_true',help='Require carrier phase-gated resets + gimbal homing')
    ap.add_argument('--particles',type=int,default=20)
    ap.add_argument('--rounds',type=int,default=11,help='Total evaluation rounds INCLUDING initial population')
    ap.add_argument('--pilot-evals',type=int,default=10)
    ap.add_argument('--weights',type=float,nargs=4,default=list(DEFAULT_WEIGHTS),metavar=('MAE','VOL','ACC','ZC'))
    ap.add_argument('--normalizers',type=Path,default=Path.home()/'hawkeye_x3_ws/rosbags/npso_normalizers.json')
    ap.add_argument('--duration',type=float,default=15.0)
    ap.add_argument('--target-x',type=float,default=80.0)
    ap.add_argument('--target-y',type=float,default=28.0)
    ap.add_argument('--tolerance',type=float,default=5.0)
    ap.add_argument('--position-timeout',type=float,default=40.0)
    ap.add_argument('--max-retries',type=int,default=2)
    ap.add_argument('--controller',type=Path,default=Path.home()/'Downloads/hawkeye_gazebo_sspd_controller_safe.py')
    ap.add_argument('--qos',type=Path,default=Path.home()/'hawkeye_x3_ws/rosbags/qos_override.yaml')
    ap.add_argument('--output',type=Path,default=Path.home()/'hawkeye_x3_ws/rosbags')
    ap.add_argument('--resume',type=Path,default=None,help='Optimization directory containing checkpoint.json')
    args=ap.parse_args()
    if args.mode=='selftest':
        selftest()
        return
    if args.particles<1 or args.rounds<1 or args.pilot_evals<1 or args.duration<1.0:
        ap.error('Invalid optimization budget / duration')
    if args.mode=='optimize' and args.particles<2:
        ap.error('Optimization requires at least two particles')
    if not all(np.isfinite(args.weights)) or any(w<0 for w in args.weights) or sum(args.weights)<=0:
        ap.error('Weights must be finite nonnegative numbers with at least one positive weight')
    args.controller=args.controller.expanduser().resolve()
    args.qos=args.qos.expanduser().resolve()
    args.normalizers=args.normalizers.expanduser().resolve()
    args.output=args.output.expanduser().resolve()
    args.output.mkdir(parents=True,exist_ok=True)
    if not args.controller.is_file() or not args.qos.is_file():
        ap.error('Safe controller or QoS file missing. Use the paths from the working 3-trial experiment.')
    if args.mode=='smoke':
        smoke_dir=args.output/('npso_smoke_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
        smoke_dir.mkdir(parents=True,exist_ok=False)
        rclpy,node,run_trial=prepare_ros(args)
        try:
            raw,folder,meta=evaluate_live(node,run_trial,args,np.array([9.6348,.1508,600.2994]),
                                          smoke_dir,1)
            log('SMOKE PASS: '+json.dumps({'raw_metrics':raw,'trial_folder':folder},indent=2))
        finally:
            node.hold_zero()
            node.destroy_node()
            rclpy.shutdown()
    elif args.mode=='pilot':
        calibrate(args)
    else:
        optimize(args)


if __name__=='__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted; controller shutdown depends on trial cleanup and adapter deadman.',file=sys.stderr)
        sys.exit(130)
