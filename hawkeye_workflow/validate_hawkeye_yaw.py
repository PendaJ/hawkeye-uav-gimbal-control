#!/usr/bin/env python3
"""Out-of-sample validation of Hawkeye yaw model; NO parameter refitting.

Uses independent command and orientation measurements and the numerical
simulation convention of fit_hawkeye_yaw.py. Saves a plot and summary JSON.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lfilter

BASE = Path.home() / 'hawkeye_x3_ws/rosbags/hawkeye_workflow'
DEFAULT = BASE / 'yaw_identification_20261106_234036' / 'yaw_response.csv'

def load_series(rows, source, col):
    a = sorted((float(r['time_s']), float(r[col]))
               for r in rows if r['source'] == source)
    if len(a) < 2:
        raise ValueError(f'Not enough {source} samples')
    a = np.asarray(a, dtype=float)
    if not np.all(np.isfinite(a)) or np.any(np.diff(a[:, 0]) < 0):
        raise ValueError(f'Invalid timestamps/values in {source}')
    return a[:,0], a[:,1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_path', nargs='?', type=Path, default=DEFAULT)
    parser.add_argument('--K', type=float, default=0.96675)
    parser.add_argument('--tau', type=float, default=0.00913)
    parser.add_argument('--delay', type=float, default=0.01205)
    args = parser.parse_args()
    if args.K <= 0 or args.tau <= 0 or args.delay < 0:
        parser.error('K>0, tau>0 and delay>=0 required')
    path = args.csv_path.expanduser()
    with path.open(newline='') as f:
        rows = list(csv.DictReader(f))
    tc, uc = load_series(rows,'command','command_yaw_rad_s')
    tf, yf = load_series(rows,'feedback','yaw_deg')
    end = min(tf.max(), tc.max(), 6.8)
    dt = .01  # exactly the fitting script's discrete simulation step
    t = np.arange(0, end, dt)
    if len(t) < 20:
        raise ValueError('Insufficient time coverage')
    meas = np.interp(t,tf,yf)

    shifted = t - args.delay
    idx = np.searchsorted(tc, shifted, side='right') - 1
    cmd = np.where(idx >= 0, uc[np.maximum(0,idx)], 0.0)
    alpha = np.exp(-dt/args.tau)
    vel = lfilter([1-alpha], [1,-alpha], cmd)
    displacement = args.K * np.rad2deg(np.cumsum(vel)*dt)

    # Align only the unknown initial angular position using the pre-excitation
    # baseline; the dynamic parameters K, tau, and delay are unchanged.
    baseline = (t >= .05) & (t <= .65)
    if not np.any(baseline):
        raise ValueError('No baseline to estimate initial yaw offset')
    offset = float(np.median(meas[baseline] - displacement[baseline]))
    pred = offset + displacement
    resid = pred - meas
    rmse = float(np.sqrt(np.mean(resid**2)))
    mae = float(np.mean(np.abs(resid)))
    p95 = float(np.percentile(np.abs(resid),95))
    range_deg = float(np.ptp(meas))
    print('INDEPENDENT MODEL VALIDATION (no refitting)')
    print('CSV:',path)
    print(f'Fixed K={args.K:.5f}, tau={args.tau:.5f} s, L={args.delay:.5f} s')
    print(f'Comparison: {t[0]:.3f} to {t[-1]:.3f} s ({len(t)} samples at {1/dt:.0f} Hz)')
    print(f'Offset from baseline only: {offset:.5f} deg')
    print(f'Validation RMSE: {rmse:.5f} deg')
    print(f'Validation MAE:  {mae:.5f} deg')
    print(f'Absolute-error P95: {p95:.5f} deg')
    print(f'Measured yaw range: {range_deg:.5f} deg')
    print(f'Normalized RMSE / measured range: {rmse/range_deg*100:.3f} %' if range_deg>0 else 'Normalized RMSE: unavailable')
    print('NOTE: Effective delay and lag remain subject to timestamp/solver uncertainty.')

    fig, axes = plt.subplots(3,1,figsize=(10,8),sharex=True)
    axes[0].step(tc,uc,where='post',label='Published yaw-rate input')
    axes[0].set_ylabel('Command (rad/s)')
    axes[1].plot(t,meas,label='Measured yaw angle',linewidth=1.8)
    axes[1].plot(t,pred,'--',label='Prediction from first trial',linewidth=1.8)
    axes[1].set_ylabel('Yaw (deg)')
    axes[1].legend()
    axes[2].plot(t,resid,label='Prediction minus measurement')
    axes[2].axhline(0,color='gray',linestyle='--',linewidth=1)
    axes[2].set_ylabel('Residual (deg)')
    axes[2].set_xlabel('Time (s)')
    for ax in axes: ax.grid(True,alpha=.3)
    fig.suptitle('Yaw model: independent pulse validation')
    fig.tight_layout()
    out = path.parent/'yaw_model_validation.png'
    fig.savefig(out,dpi=180)
    summary = {'source_csv':str(path),'refitted':False, 'K':args.K,
               'tau_s':args.tau,'delay_s':args.delay,'baseline_offset_deg':offset,
               'rmse_deg':rmse,'mae_deg':mae,'abs_error_p95_deg':p95,
               'yaw_range_deg':range_deg,'rmse_pct_yaw_range':100*rmse/range_deg if range_deg else None,
               'comparison_duration_s':float(t[-1]),'samples':len(t)}
    (path.parent/'yaw_model_validation.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('Saved:',out)
    print('Saved:',path.parent/'yaw_model_validation.json')
    plt.show()

if __name__=='__main__':
    main()
