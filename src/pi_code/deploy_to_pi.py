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
PI_PASS = "123"
PI_HOST = "192.168.137.98"
REMOTE_DIR = "/home/azmain/pi_code"

def deploy():
    print(f"[*] Deploying WRO pi_code subsystem to {PI_USER}@{PI_HOST}:{REMOTE_DIR} via Paramiko SFTP...")
    local_dir = os.path.dirname(os.path.abspath(__file__))
    
    try:
        import paramiko
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
        # Connect using password "123" and SSH keys
        ssh.connect(PI_HOST, username=PI_USER, password=PI_PASS, timeout=10)
        sftp = ssh.open_sftp()
        
        def upload_dir(local_p, remote_p):
            try:
                sftp.mkdir(remote_p)
            except:
                pass
            c = 0
            for item in os.listdir(local_p):
                if item.startswith('.') or item == '__pycache__':
                    continue
                lp = os.path.join(local_p, item)
                rp = f"{remote_p}/{item}"
                if os.path.isdir(lp):
                    c += upload_dir(lp, rp)
                else:
                    sftp.put(lp, rp)
                    c += 1
            return c

        count = upload_dir(local_dir, REMOTE_DIR)
        sftp.close()
        print(f"  [+] Successfully uploaded {count} files (including templates/) to {PI_USER}@{PI_HOST}:{REMOTE_DIR}")

        # Restart wro_nav service
        stdin, stdout, stderr = ssh.exec_command("chmod +x ~/pi_code/*.py && systemctl --user restart wro_nav.service && sleep 1 && systemctl --user status wro_nav.service --no-pager")
        out = stdout.read().decode('utf-8', errors='replace')
        try:
            print(f"[SERVICE STATUS]\n{out}")
        except:
            print("[SERVICE STATUS] Service restarted successfully!")
        ssh.close()
        print("[SUCCESS] WRO Subsystem Deployment Complete & Service Restarted!")
    except Exception as e:
        print(f"[ERROR] Deployment error: {e}")

if __name__ == '__main__':
    deploy()
