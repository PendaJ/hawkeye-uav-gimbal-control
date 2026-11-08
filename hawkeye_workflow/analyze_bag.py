#!/usr/bin/env python3
"""Extract controller-active-window tracking metrics from a ROS 2 Humble db3 bag.

Uses message acquisition timestamps. Volatility / second difference are
per YOLO observation, not physical-time-normalized derivatives.
"""
import csv
import json
import math
from pathlib import Path
import sqlite3
import statistics


def _p95(x):
    s=sorted(x)
    return s[math.ceil(.95*len(s))-1]

def _zc(values):
    signs=[1 if x>0 else -1 for x in values if x!=0]
    return sum(a!=b for a,b in zip(signs,signs[1:]))

def analyze(path):
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import Point, Twist
    # New trials: path is run directory or its nested bag/.
    # Older manual trials: path itself contains the *.db3 file.
    run_dir = (path.parent if path.name == 'bag' and (path.parent/'run.json').exists()
               else path if path.is_dir() else path.parent)
    cfg=json.loads((run_dir/'run.json').read_text()) if (run_dir/'run.json').exists() else {}
    bag=Path(cfg.get('bag',path))
    if not bag.exists() and path.is_dir() and list(path.glob('*.db3')): bag=path
    files=list(bag.glob('*.db3'))
    if not files: raise RuntimeError(f'No ROS bag SQLite .db3 found in {bag}')
    if len(files)>1: raise RuntimeError('Multi-file rosbag not supported; use ros2 bag convert first')
    conn=sqlite3.connect('file:'+str(files[0])+'?mode=ro',uri=True)
    topics={name:i for i,name in conn.execute('SELECT id,name FROM topics')}
    for name in ('/target_coord','/cam_vel'):
        if name not in topics: raise RuntimeError(f'Missing {name} in {files[0]}')
    firstbag,lastbag=conn.execute('SELECT MIN(timestamp),MAX(timestamp) FROM messages').fetchone()
    if firstbag is None: raise RuntimeError('Bag contains no messages')
    commands=[]
    nonzero=[]
    for t,payload in conn.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',(topics['/cam_vel'],)):
        m=deserialize_message(payload,Twist)
        pitch=float(m.angular.x); yaw=float(m.angular.y)
        commands.append((t,pitch,yaw))
        if abs(pitch)>1e-5 or abs(yaw)>1e-5: nonzero.append(t)
    if cfg.get('control_window_start_ns') and cfg.get('control_window_end_ns'):
        t0,t1=cfg['control_window_start_ns'],cfg['control_window_end_ns']
        mode='controller-reported-timed-window'
    elif nonzero:
        t0,t1=nonzero[0],nonzero[-1]
        mode='nonzero-command-span'
    elif cfg.get('controller')=='off' and cfg.get('controller_process_start_ns',0) and cfg.get('controller_process_end_ns',0):
        t0,t1=cfg['controller_process_start_ns'],cfg['controller_process_end_ns']
        mode='open-loop-clock-window'
    elif cfg.get('controller_process_start_ns',0) and cfg.get('controller_process_end_ns',0):
        t0,t1=cfg['controller_process_start_ns'],cfg['controller_process_end_ns']
        mode='process-clock-fallback-WARNING'
    else:
        raise RuntimeError('No nonzero commands and no recorded control window: cannot infer active interval')
    samples=[]; messages=0
    for t,payload in conn.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp',
                                  (topics['/target_coord'],t0,t1)):
        m=deserialize_message(payload,Point)
        messages+=1
        if float(m.z)>0.5 and all(math.isfinite(v) for v in (m.x,m.y)):
            ex=float(m.x); ey=float(m.y)
            samples.append((t,ex,ey,math.hypot(ex,ey)))
    conn.close()
    if len(samples)<3: raise RuntimeError(f'Only {len(samples)} valid detections: cannot compute tracking metrics')
    x=[v[1] for v in samples]; y=[v[2] for v in samples]; r=[v[3] for v in samples]
    dt=[(b[0]-a[0])/1e9 for a,b in zip(samples,samples[1:])]
    dr=[b-a for a,b in zip(r,r[1:])]
    d2r=[r[i+2]-2*r[i+1]+r[i] for i in range(len(r)-2)]
    duration=(t1-t0)/1e9
    summary={
        'source_rosbag':str(bag), 'window_method':mode,
        'control_start_s_from_bag':round((t0-firstbag)/1e9,3),
        'control_end_s_from_bag':round((t1-firstbag)/1e9,3),
        'control_span_s':round(duration,3),
        'window_target_messages':messages,'valid_detections':len(samples),
        'valid_fraction_of_published_messages':round(len(samples)/max(messages,1),4),
        'valid_detection_rate_hz':round(len(samples)/max(duration,1e-6),2),
        'mean_detection_interval_s':round(statistics.mean(dt),4),
        'std_detection_interval_s':round(statistics.pstdev(dt),4),
        'mae_x_px':round(statistics.mean(map(abs,x)),3),
        'mae_y_px':round(statistics.mean(map(abs,y)),3),
        'mean_radial_error_px':round(statistics.mean(r),3),
        'radial_rmse_px':round(math.sqrt(statistics.mean(v*v for v in r)),3),
        'radial_p95_px':round(_p95(r),3),
        'radial_max_px':round(max(r),3),
        'radial_volatility_mean_abs_delta_px_per_sample':round(statistics.mean(map(abs,dr)),4),
        'radial_volatility_std_delta_px_per_sample':round(statistics.pstdev(dr),4),
        'radial_mean_abs_second_difference_px_per_sample2':round(statistics.mean(map(abs,d2r)),4),
        'x_zero_crossings':_zc(x),'y_zero_crossings':_zc(y),
        'nonzero_command_count':len(nonzero),
        'camera_sampling_note':'Raw sample-to-sample metrics, not time-normalized; compare at matched detection rates.',
        'initialization':'phase gated + gimbal homed if run.json states repro_method; inspect per-trial status',
        'carrier_end_state':cfg.get('carrier_after_state'),
        'metric_definition':'volatility=mean(abs(diff(radial_error))); acceleration=mean(abs(diff(radial_error,n=2)))',
        'limitations':'Observation-spaced smoothness is not time-normalized; report camera sampling and carrier sim elapsed.'
    }
    out=run_dir/'analysis'
    out.mkdir(parents=True,exist_ok=True)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (out/'tracking.csv').open('w',newline='') as f:
        w=csv.writer(f); w.writerow(['time_from_control_start_s','ex_px','ey_px','radial_px'])
        for t,ex,ey,e in samples:
            w.writerow([round((t-t0)/1e9,6),ex,ey,e])
    with (out/'commands.csv').open('w',newline='') as f:
        w=csv.writer(f); w.writerow(['time_from_control_start_s','pitch_rad_s','yaw_rad_s'])
        for t,pitch,yaw in commands:
            if t0<=t<=t1:w.writerow([round((t-t0)/1e9,6),pitch,yaw])
    print('\n=== HAWKEYE TRACKING RESULTS ===')
    print(f'Window:      {mode}, {duration:.3f} s; valid {len(samples)}/{messages} samples')
    print(f'Error MAE:   x={summary["mae_x_px"]:.3f} px, y={summary["mae_y_px"]:.3f} px')
    print(f'Radial:      MAE={summary["mean_radial_error_px"]:.3f} px, '
          f'RMSE={summary["radial_rmse_px"]:.3f} px, P95={summary["radial_p95_px"]:.3f} px')
    print(f'Smoothness:  vol={summary["radial_volatility_mean_abs_delta_px_per_sample"]:.4f} px/sample, '
          f'mean|d2|={summary["radial_mean_abs_second_difference_px_per_sample2"]:.4f} px/sample²')
    print(f'Zero-cross:  x={summary["x_zero_crossings"]}, y={summary["y_zero_crossings"]}')
    print('Saved:',out/'summary.json',out/'tracking.csv',out/'commands.csv')
    if duration<1 or (cfg.get('duration_s') and not 0.9*cfg['duration_s']<=duration<=1.15*cfg['duration_s']):
        print('WARNING: inferred control span differs from configured duration; inspect controller.log and timestamps.')
    print('CAUTION: samples irregular, first/second differences per observation; compare matched sampling rates.')
    if cfg.get('carrier_after_state'):
        print('Carrier sim elapsed at controller exit:',cfg['carrier_after_state'].get('elapsed'),'s')
    return summary

if __name__=='__main__':
    import sys
    if len(sys.argv)!=2: raise SystemExit('Usage: python analyze_bag.py /path/to/run-or-bag')
    analyze(Path(sys.argv[1]).expanduser())
