#!/usr/bin/env python3
"""Start/stop the Mac serial owner and Docker physical-arm RViz viewer."""
import argparse
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import time
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / '.local/physical-arms'


def state():
    token = (RUNTIME/'token').read_text().strip()
    req = Request('http://127.0.0.1:18765/state', headers={'Authorization': 'Bearer '+token})
    with urlopen(req, timeout=3) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'stop', 'status'])
    parser.add_argument('--build', action='store_true')
    args = parser.parse_args()
    RUNTIME.mkdir(parents=True, exist_ok=True)
    docker = ['docker', 'compose', '-f', str(ROOT/'compose.physical.yaml')]
    if args.action == 'status':
        print(json.dumps(state(), indent=2))
        return
    if args.action == 'stop':
        try:
            token = (RUNTIME/'token').read_text().strip()
            req = Request('http://127.0.0.1:18765/command', data=b'{"action":"disarm"}', headers={'Authorization': 'Bearer '+token, 'Content-Type': 'application/json'})
            with urlopen(req, timeout=3) as response:
                json.load(response)
        except Exception as exc:
            print('Disarm request:', exc)
        subprocess.run(docker+['stop', 'physical'], cwd=ROOT, check=True)
        pidfile = RUNTIME/'host.pid'
        if pidfile.exists():
            pid = int(pidfile.read_text())
            # Check this PID still belongs to our dedicated host bridge.
            command = subprocess.run(['ps', '-p', str(pid), '-o', 'command='], text=True, capture_output=True).stdout
            if 'robowheels.arms.mac_bridge' in command or 'mac_servo_bridge.py' in command:
                os.kill(pid, signal.SIGTERM)
            pidfile.unlink()
        return
    tokenfile = RUNTIME/'token'
    if not tokenfile.exists():
        tokenfile.write_text(secrets.token_hex(32))
        tokenfile.chmod(0o600)
    if args.build:
        subprocess.run(docker+['build', 'physical'], cwd=ROOT, check=True)
    try:
        state()
        print('Using existing Mac serial bridge')
    except Exception:
        python = ROOT/'.local/mac-hardware/bin/python'
        command = [str(python), '-u', '-m', 'robowheels.arms.mac_bridge', '--config', str(ROOT/'.local/config/arms/servo_bridge.json'), '--zeros', str(ROOT/'.local/config/arms/servo_zero_offsets.json'), '--token-file', str(tokenfile)]
        config_dir = ROOT/'.local/config/arms'
        for name in ('servo_bridge.json', 'servo_zero_offsets.json'):
            if not (config_dir/name).is_file():
                raise RuntimeError(f'Missing local {name}; see docs/hardware/MAC_ROS_ARMS.md for setup')
        env = dict(os.environ)
        env['PYTHONPATH'] = str(ROOT/'ros2_ws/src/robowheels') + (os.pathsep+env['PYTHONPATH'] if env.get('PYTHONPATH') else '')
        with (RUNTIME/'host.log').open('ab') as log:
            process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        (RUNTIME/'host.pid').write_text(str(process.pid))
        for _ in range(40):
            if process.poll() is not None:
                raise RuntimeError('Serial bridge exited; see .local/physical-arms/host.log')
            try:
                state()
                break
            except Exception:
                time.sleep(.25)
        else:
            raise RuntimeError('Serial bridge not ready; see host.log')
    subprocess.run(docker+['up', '-d', 'physical'], cwd=ROOT, check=True)
    print('Physical RViz: http://localhost:16082/vnc.html?autoconnect=1&resize=scale')
    print('Outputs start DISARMED. Only measured feedback drives the model.')


if __name__ == '__main__':
    main()
