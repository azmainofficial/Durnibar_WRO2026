#!/usr/bin/env python3
"""
deploy_to_pi.py – Sync pi_code folder to Raspberry Pi 5 via SSH/SCP
Target: azmain@192.168.1.101
Destination: ~/pi_code/
"""

import os
import sys
import subprocess

PI_USER = "azmain"
PI_HOST = "192.168.137.44"
REMOTE_DIR = "~/pi_code"

def deploy():
    print(f"[*] Deploying WRO pi_code subsystem to {PI_USER}@{PI_HOST}:{REMOTE_DIR} ...")
    local_dir = os.path.dirname(os.path.abspath(__file__))
    
    try:
        # Create remote target directory
        mkdir_cmd = ["ssh", "-o", "StrictHostKeyChecking=accept-new", f"{PI_USER}@{PI_HOST}", f"mkdir -p {REMOTE_DIR}/templates"]
        subprocess.run(mkdir_cmd, check=True)

        # Upload pi_code directory contents
        cmd = f'scp -r -o StrictHostKeyChecking=accept-new "{local_dir}/." {PI_USER}@{PI_HOST}:{REMOTE_DIR}/'
        res = subprocess.run(cmd, shell=True, capture_output=True, text=True)

        if res.returncode == 0 or "camera_test" in res.stdout:
            print(f"  [+] Successfully deployed files to {PI_USER}@{PI_HOST}:{REMOTE_DIR}")
            chmod_cmd = ["ssh", "-o", "StrictHostKeyChecking=accept-new", f"{PI_USER}@{PI_HOST}", f"chmod +x {REMOTE_DIR}/*.py"]
            subprocess.run(chmod_cmd, check=True)
            print("[SUCCESS] WRO Subsystem Deployment Complete!")
        else:
            print(f"[!] scp output: {res.stdout}\nErrors: {res.stderr}")
    except Exception as e:
        print(f"[ERROR] Deployment error: {e}")

if __name__ == '__main__':
    deploy()
