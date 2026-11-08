#!/usr/bin/env python3
"""A small supervisor for the verified Hawkeye X3 Gazebo + ROS2 setup.

Does not change controller, gimbal adapter, perception or NPSO algorithm.
"""
import argparse
import csv
import random
import datetime
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
WS = Path(os.environ.get('HAWKEYE_WS', str(Path.home() / 'hawkeye_x3_ws')))
DATA = WS / 'rosbags' / 'hawkeye_workflow'
STATE = DATA / 'runtime.json'
PY_VENV = WS / '.venv_yolo' / 'bin' / 'python'
CTRL = Path.home() / 'Downloads' / 'hawkeye_gazebo_sspd_controller_safe.py'
CTRL_ZN_PD = ROOT / 'hawkeye_gazebo_zn_pd_controller.py'
KIN = WS / 'hawkeye_kinematic_carrier'
WORLD = WS / 'src' / 'drone_ttc_sim' / 'worlds'
BRIDGE_SPECS = [
    '/drone/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
    '/model/x3/gimbal_joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
    '/hawkeye/carrier/state@std_msgs/msg/String[gz.msgs.StringMsg',
    '/model/x3/joint/gimbal_yaw_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
    '/model/x3/joint/gimbal_pitch_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
]

def run(cmd, *, timeout=8, check=False, env=None):
    return subprocess.run(cmd, text=True, capture_output=True, timeout=timeout, check=check, env=env)

def safe_run(cmd, timeout=8):
    try: return run(cmd, timeout=timeout)
    except (subprocess.TimeoutExpired, FileNotFoundError): return None

def fail(s):
    raise RuntimeError(s)

def load_state():
    if STATE.exists():
        return json.loads(STATE.read_text())
    return None

def write_state(s):
    DATA.mkdir(parents=True, exist_ok=True)
    STATE.write_text(json.dumps(s, indent=2))

def tick(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
    except (FileNotFoundError, ValueError, IndexError, PermissionError):
        return None

def alive(entry):
    pid = entry['pid']
    return tick(pid) is not None and tick(pid) == entry.get('start_ticks')

def launch(name, cmd, log, s, env=None):
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open('ab', buffering=0) as out:
        p = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT,
                             start_new_session=True, cwd=str(WS), env=env)
    item = {'pid': p.pid, 'start_ticks': tick(p.pid), 'cmd': cmd, 'log': str(log)}
    s['processes'][name] = item
    write_state(s)
    print(f'  {name}: PID {p.pid} (log: {log})', flush=True)
    time.sleep(0.5)
    if p.poll() is not None:
        fail(f'{name} exited immediately; inspect {log}')
    return p

def stop_process(name, entry, grace=7):
    if not alive(entry):
        print(f'  {name}: no live managed process', flush=True)
        return
    try: os.killpg(entry['pid'], signal.SIGINT)
    except ProcessLookupError: return
    till=time.monotonic()+grace
    while time.monotonic()<till and alive(entry): time.sleep(0.1)
    if alive(entry):
        try: os.killpg(entry['pid'], signal.SIGTERM)
        except ProcessLookupError: pass
        time.sleep(0.5)
    if alive(entry):
        try: os.killpg(entry['pid'], signal.SIGKILL)
        except ProcessLookupError: pass
    print(f'  {name}: stop requested', flush=True)

def gazebo_running():
    r=safe_run(['gz','topic','-l'], timeout=5)
    if r is None or r.returncode!=0: return False
    return '/drone/camera/image_raw' in r.stdout or '/world/default/pose/info' in r.stdout

def check_static_dependencies():
    for path in (KIN/'prepare_world.py', KIN/'build/libHawkeyeCarrierMotion.so',
                 CTRL, PY_VENV):
        if not path.exists(): fail(f'Missing {path}. Complete existing setup/build first.')
    for cmd in (['ros2','pkg','executables','hawkeye_x3_gimbal'],
                ['ros2','pkg','executables','ros_gz_bridge']):
        result=run(cmd)
        if result.returncode: fail(f'Missing installed ROS package: {shlex.join(cmd)}')

def up(args):
    s=load_state()
    if s and any(alive(p) for p in s.get('processes',{}).values()):
        fail('A managed stack is already running. Use `./hawkeye status` or `./hawkeye stop`.')
    if gazebo_running():
        fail('Another Gazebo world is running. Stop older manual Gazebo / launch terminals first; this command will not duplicate a world.')
    existing=safe_run(['ros2','node','list'])
    if existing and any(n in existing.stdout.splitlines() for n in
                        ('/hawkeye_gimbal_adapter','/hawkeye_person_tracker')):
        fail('Existing tracker/adapter nodes detected. Stop older manual ROS nodes first, then retry.')
    check_static_dependencies()
    DATA.mkdir(parents=True, exist_ok=True)
    now=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    session=DATA/f'session_{args.mode}_{now}'
    session.mkdir(parents=True)
    s={'mode':args.mode, 'seed':args.seed, 'session':str(session), 'processes':{}, 'created':now}
    write_state(s)
    try:
        res=run([sys.executable, str(KIN/'prepare_world.py'), '--mode', args.mode, '--triggered'], timeout=10)
        if res.returncode: fail('World preparation failed:\n'+res.stdout+res.stderr)
        print(res.stdout.strip(), flush=True)
        env=os.environ.copy()
        env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(KIN/'build')+':'+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
        env['GZ_SIM_RESOURCE_PATH']=str(WS/'src/drone_ttc_sim/models')+':'+env.get('GZ_SIM_RESOURCE_PATH','')
        launch('gazebo',['gz','sim','-r',str(WORLD/f'hawkeye_gimbal_{args.mode}.world')],session/'gazebo.log',s,env)
        # Give the simulation a moment to load plugins; do not assume instant readiness.
        deadline=time.monotonic()+45
        while time.monotonic()<deadline and not gazebo_running():
            if not alive(s['processes']['gazebo']): fail('Gazebo terminated; see gazebo.log')
            time.sleep(1)
        if not gazebo_running(): fail('Gazebo camera topic did not appear in 45 s; inspect gazebo.log')
        launch('bridge',['ros2','run','ros_gz_bridge','parameter_bridge',*BRIDGE_SPECS],session/'bridge.log',s)
        launch('adapter',['ros2','run','hawkeye_x3_gimbal','gimbal_adapter'],session/'adapter.log',s)
        # Run YOLO with the same venv interpreter and source tree as working manual steps.
        tracker_env=os.environ.copy()
        tracker_env['PYTHONHASHSEED']=str(args.seed)
        launch('tracker',[str(PY_VENV),str(ROOT/'tracker_seed.py'),str(args.seed)],session/'tracker.log',s,tracker_env)
        print('Waiting for live ROS topics / tracker warm-up (up to 90 s) ...', flush=True)
        deadline=time.monotonic()+90
        needed=('/drone/camera/image_raw','/target_coord','/current_orientation',
                '/hawkeye/carrier/state')
        ready=False
        while time.monotonic()<deadline:
            for name,entry in s['processes'].items():
                if not alive(entry): fail(f'{name} stopped unexpectedly; inspect {entry["log"]}')
            info=safe_run(['ros2','topic','list'])
            topics=set(info.stdout.splitlines()) if info and info.returncode==0 else set()
            if all(n in topics for n in needed):
                # Topic discovery isn't proof of messages; use one-shot checks below.
                ready=True; break
            time.sleep(1)
        if not ready: fail('Required topics not discovered in 90 s; inspect bridge/adapter/tracker logs')
        for topic in ('/target_coord','/current_orientation'):
            p=safe_run(['ros2','topic','echo',topic,'--once'],timeout=15)
            if p is None or p.returncode: fail(f'No live message on {topic}; inspect service logs')
        # Probe carrier state, not just topic presence; require the new phase-gated plugin.
        sys.path.insert(0,str(WS/'hawkeye_npso_gazebo'))
        from repro_tools import state as carrier_state
        carrier=carrier_state()
        if not carrier.get('triggered',False) or carrier.get('mode')!=args.mode:
            fail('Carrier plugin wrong mode or not phase gated. Rebuild libHawkeyeCarrierMotion.so')
        print('CARRIER:',carrier,flush=True)
        print(f'READY: {args.mode} world with phase-gated carrier, camera, seeded YOLO, adapter and bridges.')
        print('Use: ./hawkeye trial sspd 60  (or: ./hawkeye trial off 60)')
    except BaseException:
        print('Startup failed; cleaning up managed processes...', file=sys.stderr)
        for name, entry in reversed(list(s['processes'].items())): stop_process(name,entry)
        if STATE.exists(): STATE.unlink()
        raise

def status(_args):
    s=load_state()
    if not s:
        print('No managed stack. Use ./hawkeye start hold|lateral|combined')
        return
    print('Mode:',s['mode'],'\nSession:',s['session'])
    for name,entry in s['processes'].items():
        print(f'{name:10} {"RUNNING" if alive(entry) else "STOPPED"} pid={entry["pid"]}')
    for topic in ('/drone/camera/image_raw','/target_coord','/current_orientation','/cam_vel'):
        result=safe_run(['ros2','topic','info',topic],timeout=4)
        if result and result.returncode==0:
            pubs=re.search(r'Publisher count:\s*(\d+)',result.stdout)
            subs=re.search(r'Subscription count:\s*(\d+)',result.stdout)
            print(f'{topic}: pub={pubs.group(1) if pubs else "?"}, sub={subs.group(1) if subs else "?"}')
        else: print(f'{topic}: unavailable')

def stop(_args):
    s=load_state()
    if not s:
        print('No managed stack (manual terminals are not modified).')
        return
    if (DATA/'trial.lock').exists():
        fail('A trial is running (trial.lock exists). Finish or interrupt the trial before stopping the stack.')
    for name,entry in reversed(list(s['processes'].items())):
        stop_process(name,entry)
    STATE.unlink(missing_ok=True)
    print('Stopped managed processes only. Logs and bags retained.')

def extract_counts(s):
    r=safe_run(['ros2','topic','info','/cam_vel'],timeout=6)
    if not r or r.returncode: fail('No /cam_vel topic: is adapter running?')
    pm=re.search(r'Publisher count:\s*(\d+)',r.stdout)
    sm=re.search(r'Subscription count:\s*(\d+)',r.stdout)
    return int(pm.group(1)) if pm else -1, int(sm.group(1)) if sm else -1

def trial(args):
    s=load_state()
    if not s: fail('Start stack first: ./hawkeye start lateral --seed 42')
    for name,ent in s['processes'].items():
        if not alive(ent):fail(f'{name} stopped; inspect {ent["log"]}')
    lock=DATA/'trial.lock'
    if lock.exists():fail('trial.lock exists: another trial or optimizer may be running')
    pubs,subs=extract_counts(s)
    if pubs>0:fail(f'/cam_vel has {pubs} existing publishers; stop competing controllers')
    if subs<1:fail('No gimbal adapter subscriber on /cam_vel')
    if args.duration<=0 or args.duration>3600:fail('Duration out of range')
    gains={'sspd':(9.6348,0.1508,600.2994),'zn_sspd':(7.68,0.48,632.0),
           'zn_pd':(7.68,0.48,632.0)}
    kp,kd,wo=gains.get(args.controller,(args.kp,args.kd,args.wo))
    if args.controller!='off' and not (0<kp<=100 and 0<=kd<=20 and 0<wo<2000):
        fail('Unsafe gains: Kp(0,100], Kd[0,20], omega(0,2000)')
    controller_file = CTRL_ZN_PD if args.controller in ('zn_pd','pd_custom') else CTRL
    if args.controller in ('zn_pd','pd_custom') and not controller_file.is_file():
        fail(f'Missing classical PD controller: {controller_file}')
    now=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir=DATA/f'{s["mode"]}_{args.controller}_{now}'
    lock.write_text(f'{os.getpid()} {run_dir}\n')
    try:
        cmd=[str(PY_VENV),str(ROOT/'repro_trial.py'),
             '--path',str(run_dir),'--world',s['mode'],
             '--controller',args.controller,'--duration',str(args.duration),
             '--initial-x',str(args.initial_x),'--initial-y',str(args.initial_y),
             '--tolerance',str(args.tolerance), '--seed',str(s['seed']),
             '--kp',str(kp),'--kd',str(kd),'--wo',str(wo),
             '--python',str(PY_VENV), '--controller-file',str(controller_file),
             '--plugin',str(KIN/'build/libHawkeyeCarrierMotion.so'),
             '--qos',str(WS/'rosbags/qos_override.yaml')]
        env=os.environ.copy()
        env['PYTHONPATH']=str(WS/'hawkeye_npso_gazebo')+':'+env.get('PYTHONPATH','')
        print('REPRODUCIBLE TRIAL:',args.controller,'world',s['mode'],
              'duration',args.duration,'seed',s['seed'],'gains',(kp,kd,wo),flush=True)
        p=subprocess.run(cmd,env=env)
        if p.returncode:fail(f'Trial failed, logs retained at {run_dir}')
        print('Recorded:',run_dir,flush=True)
        analyze(argparse.Namespace(path=str(run_dir)))
    finally:
        lock.unlink(missing_ok=True)


def analyze(args):
    if args.path:
        path=Path(args.path).expanduser()
    else:
        runs=sorted((p for p in DATA.iterdir() if p.is_dir() and (p/'run.json').exists()),
                    key=lambda p:p.stat().st_mtime,reverse=True) if DATA.exists() else []
        if not runs: fail('No trial found. Supply a bag/run path: ./hawkeye analyze /path/to/bag')
        path=runs[0]
    import analyze_bag
    analyze_bag.analyze(path)

def optimize(args):
    script=WS/'hawkeye_npso_gazebo/npso_gazebo.py'
    if not script.exists():fail(f'Missing optimizer: {script}')
    s=load_state()
    if args.mode!='selftest':
        if not s:fail('Start stack: ./hawkeye start lateral --seed 42')
        if any(not alive(p) for p in s['processes'].values()):fail('Managed stack incomplete')
        if (DATA/'trial.lock').exists():fail('Trial or optimization already running')
        pubs,_=extract_counts(s)
        if pubs:fail('Another /cam_vel publisher is running')
    world=s['mode'] if s else 'lateral'
    norm=(Path(args.normalizers).expanduser() if args.normalizers else
          DATA/f'normalizers_{world}_{args.duration:g}s.json')
    cmd=[str(PY_VENV),str(script),'--mode',args.mode]
    if args.mode!='selftest':
        cmd+=['--world-mode',world,'--repro','--duration',str(args.duration),
              '--normalizers',str(norm)]
    if args.mode=='smoke':cmd+=['--seed',str(args.seed)]
    if args.mode=='pilot':cmd+=['--pilot-evals',str(args.evals),'--seed',str(args.seed)]
    if args.mode=='optimize':cmd+=['--particles',str(args.particles),
                                   '--rounds',str(args.rounds),'--seed',str(args.seed)]
    if args.resume:cmd+=['--resume',str(Path(args.resume).expanduser())]
    print('Running:',shlex.join(cmd),flush=True)
    lock=DATA/'trial.lock'
    if args.mode!='selftest':lock.write_text(f'{os.getpid()} NPSO {world}\n')
    try:
        p=subprocess.run(cmd,cwd=str(script.parent))
    finally:
        if args.mode!='selftest':lock.unlink(missing_ok=True)
    if p.returncode:fail(f'Optimizer returned {p.returncode}; see saved optimizer files')

def suite(args):
    """Fixed-seed randomized controller-order comparison with matched starts."""
    if args.repeats<1 or args.repeats>20 or not 0<args.duration<=3600:
        fail('Invalid suite repeats or duration')
    s=load_state()
    if not s:fail('Start stack first: ./hawkeye start lateral --seed 42')
    best=None
    if args.best_json:
        path=Path(args.best_json).expanduser()
        best=json.loads(path.read_text())
        best={k:float(best[k]) for k in ('kp','kd','omega_o')}
    rng=random.Random(args.seed)
    suite_path=DATA/f'suite_{s["mode"]}_seed{args.seed}_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    suite_path.mkdir(parents=True,exist_ok=False)
    rows=[]
    details={'mode':s['mode'],'seed':args.seed,'tracker_seed':s['seed'],
             'repeats':args.repeats,'duration_s':args.duration,
             'initial_offset_px':[args.initial_x,args.initial_y],
             'best_parameters':best,'trials':[]}
    for rep in range(1,args.repeats+1):
        labels=['off','zn_sspd','sspd']
        rng.shuffle(labels)
        print(f'REPEAT {rep}: fixed-seed randomized order {labels}',flush=True)
        for label in labels:
            trial_args=argparse.Namespace(controller=label,duration=args.duration,
                                          initial_x=args.initial_x,initial_y=args.initial_y,
                                          tolerance=args.tolerance,
                                          kp=9.6348,kd=.1508,wo=600.2994)
            if label=='sspd' and best:
                trial_args.controller='custom'
                trial_args.kp=best['kp']; trial_args.kd=best['kd'];trial_args.wo=best['omega_o']
            existing={p for p in DATA.glob(f'{s["mode"]}_{trial_args.controller}_*') if p.is_dir()}
            trial(trial_args)
            new=[p for p in DATA.glob(f'{s["mode"]}_{trial_args.controller}_*')
                 if p.is_dir() and p not in existing and (p/'run.json').is_file()]
            if len(new)!=1:
                fail('Could not locate exactly one new run after trial; inspect recorded data')
            path=new[0];summary=json.loads((path/'analysis/summary.json').read_text())
            row={'rep':rep,'label':('npso_gazebo' if label=='sspd' and best else label),
                 'run_directory':str(path),
                 'mae_x_px':summary['mae_x_px'],'mae_y_px':summary['mae_y_px'],
                 'mean_radial_error_px':summary['mean_radial_error_px'],
                 'radial_rmse_px':summary['radial_rmse_px'],
                 'radial_p95_px':summary['radial_p95_px'],
                 'radial_volatility_mean_abs_delta_px_per_sample':
                     summary['radial_volatility_mean_abs_delta_px_per_sample'],
                 'radial_mean_abs_second_difference_px_per_sample2':
                     summary['radial_mean_abs_second_difference_px_per_sample2'],
                 'x_zero_crossings':summary['x_zero_crossings'],
                 'y_zero_crossings':summary['y_zero_crossings'],
                 'valid_detections':summary['valid_detections'],
                 'valid_detection_rate_hz':summary['valid_detection_rate_hz']}
            rows.append(row)
            details['trials'].append(row)
            (suite_path/'suite.json').write_text(json.dumps(details,indent=2)+'\n')
            with (suite_path/'suite.csv').open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(rows[0]))
                writer.writeheader();writer.writerows(rows)
    print('SUITE COMPLETE:',suite_path/'suite.csv',flush=True)


def main():
    ap=argparse.ArgumentParser(description='Hawkeye reproducible Gazebo/ROS2 trial workflow')
    sub=ap.add_subparsers(dest='command',required=True)
    start=sub.add_parser('start',help='Start world + camera/joint bridges + adapter + YOLO')
    start.add_argument('mode',choices=('hold','lateral','combined'))
    start.add_argument('--seed',type=int,default=42)
    start.set_defaults(func=up)
    st=sub.add_parser('status');st.set_defaults(func=status)
    sp=sub.add_parser('stop');sp.set_defaults(func=stop)
    tr=sub.add_parser('trial',help='Home gimbal, match initial error, sync carrier, record, analyze')
    tr.add_argument('controller',choices=('off','sspd','zn_sspd','zn_pd','pd_custom','custom'))
    tr.add_argument('duration',type=float,nargs='?',default=60.0)
    tr.add_argument('--kp',type=float,default=9.6348)
    tr.add_argument('--kd',type=float,default=0.1508)
    tr.add_argument('--wo',type=float,default=600.2994)
    tr.add_argument('--initial-x',type=float,default=80.0)
    tr.add_argument('--initial-y',type=float,default=28.0)
    tr.add_argument('--tolerance',type=float,default=5.0)
    tr.set_defaults(func=trial)
    an=sub.add_parser('analyze',help='Analyze latest trial or specified ROS bag/run directory')
    an.add_argument('path',nargs='?')
    an.set_defaults(func=analyze)
    su=sub.add_parser('suite',help='Repeat off/ZN/SSPD in fixed-seed randomized order')
    su.add_argument('duration',type=float,nargs='?',default=60.0)
    su.add_argument('--repeats',type=int,default=3)
    su.add_argument('--seed',type=int,default=42)
    su.add_argument('--best-json',help='Path to completed Gazebo NPSO best_parameters.json')
    su.add_argument('--initial-x',type=float,default=80.0)
    su.add_argument('--initial-y',type=float,default=28.0)
    su.add_argument('--tolerance',type=float,default=5.0)
    su.set_defaults(func=suite)
    op=sub.add_parser('optimize',help='Run NPSO in active phase-gated world')
    op.add_argument('mode',choices=('selftest','smoke','pilot','optimize'))
    op.add_argument('--evals',type=int,default=10)
    op.add_argument('--particles',type=int,default=20)
    op.add_argument('--rounds',type=int,default=11)
    op.add_argument('--seed',type=int,default=42)
    op.add_argument('--resume')
    op.add_argument('--duration',type=float,default=15.0,help='At least one 12.57 s lateral period')
    op.add_argument('--normalizers')
    op.set_defaults(func=optimize)
    args=ap.parse_args()
    try: args.func(args)
    except (RuntimeError, subprocess.TimeoutExpired, OSError, ValueError) as exc:
        print('ERROR:',exc,file=sys.stderr)
        sys.exit(1)

if __name__=='__main__': main()
