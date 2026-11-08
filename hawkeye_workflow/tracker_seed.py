#!/usr/bin/env python3
"""Seed Python/NumPy/PyTorch before executing the existing tracker module."""
import os
import random
import runpy
import sys

seed=int(sys.argv[1])
os.environ['PYTHONHASHSEED']=str(seed)
random.seed(seed)
try:
    import numpy as np
    np.random.seed(seed)
except ImportError:
    pass
try:
    import torch
    torch.manual_seed(seed)
    if torch.cuda.is_available():torch.cuda.manual_seed_all(seed)
except ImportError:
    pass
sys.argv=['hawkeye_x3_perception.person_tracker', '--ros-args', '-p', 'cpu_threads:='+os.environ.get('HAWKEYE_CPU_THREADS','4')]
# HAWKEYE_YOLO_DEVICE_CONFIG: preserve seeded launcher and all existing ROS parameters.
sys.argv += ['-p', 'device:='+os.environ.get('HAWKEYE_YOLO_DEVICE','cpu')]
runpy.run_module('hawkeye_x3_perception.person_tracker',run_name='__main__',alter_sys=True)
