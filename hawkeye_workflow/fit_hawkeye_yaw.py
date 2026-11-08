#!/usr/bin/env python3
"""Fit measured Hawkeye yaw displacement to a delayed first-order velocity actuator plus integrator.

G(s) = K exp(-L s) / [s (tau s + 1)]
Identification is illustrative unless model residuals and independent validation are checked.
"""
import csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.signal import lfilter

base = Path.home() / 'hawkeye_x3_ws/rosbags/hawkeye_workflow'
path = base / 'yaw_identification_20261106_233155' / 'yaw_response.csv'
with path.open(newline='') as f:
    rows = list(csv.DictReader(f))

def read(source, column):
    a = sorted((float(r['time_s']), float(r[column])) for r in rows if r['source'] == source)
    if not a:
        raise ValueError('No samples for '+source)
    return np.asarray(a, dtype=float).T

tc, uc = read('command','command_yaw_rad_s')
tf, yf = read('feedback','yaw_deg')
mask = np.isfinite(tf) & np.isfinite(yf) & (tf >= 0) & (tf <= 5.8)
tf,yf = tf[mask],yf[mask]
if tf.size < 50: raise ValueError('Not enough feedback samples')
dt = .01
t = np.arange(0, min(5.8,tf.max()),dt)
y = np.interp(t,tf,yf)

def model(p):
    gain,tau,delay,offset = p
    shifted = t-delay
    idx = np.searchsorted(tc,shifted,side='right')-1
    cmd = np.where(idx>=0,uc[np.maximum(0,idx)],0.0)
    alpha = np.exp(-dt/max(tau,1e-6))
    velocity = lfilter([1-alpha],[1,-alpha],cmd)
    angle_deg = np.rad2deg(np.cumsum(velocity)*dt)
    return offset+gain*angle_deg

fit = least_squares(lambda p:model(p)-y, x0=[.97,.025,.015,float(y[0])],
                    bounds=([.5,.001,0,y[0]-1],[1.5,.35,.25,y[0]+1]),
                    max_nfev=2000,xtol=1e-11,ftol=1e-11,gtol=1e-11)
K,tau,delay,offset=fit.x
pred=model(fit.x)
rmse=np.sqrt(np.mean((pred-y)**2))
mae=np.mean(np.abs(pred-y))
print('Yaw identification (measured command -> yaw angle)')
print('File:',path)
print(f'Gain K     = {K:.5f} [rad/s actual / rad/s command]')
print(f'Lag tau    = {tau:.5f} s')
print(f'Delay L    = {delay:.5f} s')
print(f'RMSE       = {rmse:.5f} deg')
print(f'MAE        = {mae:.5f} deg')
print('Model: G(s) = K exp(-L s)/(s(tau s+1))')
print('CAUTION: tau and delay may trade off; validate against a second pulse duration.')
for tc_des in (.3,.4,.5):
    print(f'Illustrative P-only Kp for tau_c={tc_des:.1f} s: {1/(K*tc_des):.4f}')

fig,axs=plt.subplots(2,1,figsize=(10,7),sharex=True)
axs[0].plot(t,y,label='Measured yaw (deg)')
axs[0].plot(t,pred,'--',label='Fitted model')
axs[0].set_ylabel('Yaw angle (deg)')
axs[0].legend()
axs[1].plot(t,pred-y,label='Model - measurement')
axs[1].axhline(0,color='gray',ls='--',lw=1)
axs[1].set_ylabel('Residual (deg)')
axs[1].set_xlabel('Time (s)')
for ax in axs:ax.grid(True)
fig.tight_layout()
out=path.parent/'yaw_model_fit.png'
fig.savefig(out,dpi=180)
print('Saved plot:',out)
