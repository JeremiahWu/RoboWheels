#!/usr/bin/env python3
"""Start/stop the Mac RViz and SSH read-only telemetry relay; never drives motors."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
LOCAL=ROOT/'.local/rdk-debug'
COMPOSE=['docker','compose','-f',str(ROOT/'compose.rdk-debug.yaml')]
def default_host():
    configured=ROOT/'.local/config/rdk-host'
    return os.environ.get('ROBOWHEELS_RDK_HOST') or (configured.read_text().strip() if configured.is_file() else 'root@rdkx5.local')

SSH=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2']
def running():
    try:
        pid=int((LOCAL/'supervisor.pid').read_text());os.kill(pid,0)
        args=subprocess.check_output(['ps','-p',str(pid),'-o','command='],text=True)
        return pid if str(Path(__file__).resolve()) in args and '_relay' in args else None
    except (OSError,ValueError,subprocess.CalledProcessError):return None
def relay(host):
    active=[]
    def stop(*_):
        for proc in active:
            if proc.poll() is None:proc.terminate()
        raise SystemExit
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    command='source /opt/ros/humble/setup.bash; source /opt/tros/humble/setup.bash; export ROS_DOMAIN_ID=0; exec python3 /tmp/robowheels-topic-stream.py send'
    while True:
        active=[]
        sender=subprocess.Popen(SSH+[host,command],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE);active.append(sender)
        receiver=subprocess.Popen(COMPOSE+['exec','-T','rdk-debug','bash','-lc','source /opt/ros/humble/setup.bash; python3 /debug/ros_topic_stream.py receive'],stdin=sender.stdout);active.append(receiver)
        sender.stdout.close()
        while all(proc.poll() is None for proc in active):time.sleep(.5)
        for proc in active:
            if proc.poll() is None:proc.terminate()
        for proc in active:
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
        print('Telemetry disconnected; retrying in 3 seconds',flush=True);time.sleep(3)
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=['start','stop','status','_relay']);p.add_argument('--host',default=default_host());a=p.parse_args()
    if a.command=='_relay':relay(a.host);return
    LOCAL.mkdir(parents=True,exist_ok=True)
    pid=running()
    if a.command=='start':
        if os.environ.get('ROBOWHEELS_DEBUG_DISPLAY','web') == 'xquartz':
            subprocess.run([sys.executable,str(ROOT/'scripts/xquartz.py'),'prepare'],check=True)
        else:
            # The legacy XQuartz bind mount is unused in browser mode.
            auth=ROOT/'.local/x11/Xauthority';auth.parent.mkdir(parents=True,exist_ok=True)
            if not auth.exists():auth.touch(mode=0o600)
        subprocess.run(['scp','-q','-o','BatchMode=yes','-o','ConnectTimeout=5',str(ROOT/'scripts/ros_topic_stream.py'),a.host+':/tmp/robowheels-topic-stream.py'],check=True)
        subprocess.run(COMPOSE+['up','-d'],check=True)
        if not pid:
            with (LOCAL/'relay.log').open('ab') as log:
                proc=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'_relay','--host',a.host],stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
            (LOCAL/'supervisor.pid').write_text(str(proc.pid))
        print('RViz + read-only relay started. Browser: http://localhost:16081/vnc.html?autoconnect=1&resize=scale')
    elif a.command=='stop':
        if pid:os.kill(pid,signal.SIGTERM)
        subprocess.run(COMPOSE+['stop','rdk-debug'],check=True)
    else:
        print('Telemetry supervisor:',pid or 'stopped');subprocess.run(COMPOSE+['ps'],check=True)
if __name__=='__main__':main()
