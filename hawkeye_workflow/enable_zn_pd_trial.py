#!/usr/bin/env python3
"""Safely enable the zn_pd label in the existing reproducible trial runner.

Run on the Ubuntu workstation AFTER copying this script into hawkeye_workflow.
Does not modify any controller or experiment results.
"""
from pathlib import Path
import ast
import shutil
import sys
path=Path.home()/'hawkeye_x3_ws'/'hawkeye_workflow'/'repro_trial.py'
if not path.is_file():
    sys.exit(f'ERROR: missing {path}')
original=path.read_text()
old="choices=['off','sspd','zn_sspd','custom']"
new="choices=['off','sspd','zn_sspd','zn_pd','custom']"
if new in original:
    print('Already enabled:',path)
    sys.exit(0)
if original.count(old)!=1:
    sys.exit('ERROR: Unexpected repro_trial.py layout. No files changed.')
updated=original.replace(old,new)
ast.parse(updated,filename=str(path))
backup=path.with_name('repro_trial.py.before_figure5')
if backup.exists():
    sys.exit(f'ERROR: Backup already exists: {backup}; inspect before modifying.')
shutil.copy2(path,backup)
path.write_text(updated)
print('Enabled zn_pd in:',path)
print('Original backed up to:',backup)
