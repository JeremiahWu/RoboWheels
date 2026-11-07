#!/usr/bin/env python3
"""Local authenticated XQuartz bridge and native RViz lifecycle.

Only 127.0.0.1:6099 is opened. No xhost changes or XQuartz TCP preferences are
required. The current display's cookie is copied to ignored .local/x11/ only.
"""
import argparse
import os
from pathlib import Path
import select
import signal
import socket
import socketserver
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
LOCAL=ROOT/'.local/x11'
PID=LOCAL/'proxy.pid'
COOKIE=LOCAL/'Xauthority'


def display_socket():
    display=os.environ.get('DISPLAY') or subprocess.check_output(['launchctl','getenv','DISPLAY'],text=True).strip()
    if not display or not display.startswith('/'):
        raise RuntimeError('Start XQuartz first; expected its launchd UNIX display socket.')
    if not Path(display).exists():
        raise RuntimeError(f'XQuartz display socket is unavailable: {display}')
    number=display.rsplit(':',1)[-1]
    actual=Path('/tmp/.X11-unix')/('X'+number)
    return str(actual) if number.isdigit() and actual.exists() else display


def prepare():
    display=display_socket()
    LOCAL.mkdir(parents=True,exist_ok=True);LOCAL.chmod(0o700)
    number=Path(display).name[1:] if Path(display).name.startswith('X') else display.rsplit(':',1)[-1]
    auth_display=':'+number if number.isdigit() else display
    result=subprocess.run(['/opt/X11/bin/xauth','nlist',auth_display],capture_output=True,text=True,check=True)
    lines=[line for line in result.stdout.splitlines() if line.strip()]
    if not lines:
        number=display.rsplit(':',1)[-1]
        if number.isdigit():
            result=subprocess.run(['/opt/X11/bin/xauth','nlist',':'+number],capture_output=True,text=True,check=True)
            lines=[line for line in result.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError('XQuartz has no authentication cookie for this display; open an XQuartz terminal and retry.')
    temporary=COOKIE.with_suffix('.tmp')
    if temporary.exists():temporary.unlink()
    fd=os.open(temporary,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600);os.close(fd)
    # FamilyWild permits Docker's host.docker.internal alias; the secret remains
    # scoped to the already-existing XQuartz display cookie.
    records=[]
    for line in lines:
        fields=line.split()
        if len(fields)!=9:raise RuntimeError('Unexpected xauth record format')
        fields[0]='ffff'
        # The forwarded display is :99 even when XQuartz itself uses :0.
        fields[3]='0002';fields[4]='3939'
        records.append(' '.join(fields))
    data='\n'.join(records)+'\n'
    subprocess.run(['/opt/X11/bin/xauth','-f',str(temporary),'nmerge','-'],input=data,text=True,capture_output=True,check=True)
    temporary.replace(COOKIE)
    return display


def proxy_running():
    try:
        pid=int(PID.read_text());os.kill(pid,0)
        command=subprocess.run(['ps','-p',str(pid),'-o','command='],capture_output=True,text=True,check=True).stdout
        if str(Path(__file__).resolve()) not in command or ' _serve ' not in command:return None
        with socket.create_connection(('127.0.0.1',6099),timeout=.5):pass
        return pid
    except (OSError,ValueError,subprocess.CalledProcessError):return None


def start_proxy():
    display=prepare()
    pid=proxy_running()
    socket_record=LOCAL/'proxy.socket'
    if pid and socket_record.exists() and socket_record.read_text()==display:return
    if pid:
        os.kill(pid,signal.SIGTERM)
        time.sleep(.2)
    # Refuse to replace another application that owns the port.
    probe=socket.socket();probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    try:probe.bind(('127.0.0.1',6099))
    finally:probe.close()
    with (LOCAL/'proxy.log').open('ab') as log:
        process=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'_serve','--socket',display],
                                 stdout=log,stderr=log,start_new_session=True,cwd=ROOT)
    PID.write_text(str(process.pid));PID.chmod(0o600)
    socket_record.write_text(display)
    for _ in range(30):
        if proxy_running():return
        if process.poll() is not None:raise RuntimeError('XQuartz bridge failed; inspect .local/x11/proxy.log')
        time.sleep(.1)
    raise RuntimeError('Timed out starting XQuartz bridge')


def serve(path):
    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            try:
                with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as remote:
                    remote.connect(path)
                    while True:
                        readable,_,_=select.select([self.request,remote],[],[])
                        for source in readable:
                            data=source.recv(65536)
                            if not data:return
                            (remote if source is self.request else self.request).sendall(data)
            except (OSError,ConnectionError):pass
    class Server(socketserver.ThreadingTCPServer):
        allow_reuse_address=True
        daemon_threads=True
    with Server(('127.0.0.1',6099),Handler) as server:server.serve_forever()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('command',choices=['prepare','start','stop','status','build','_serve']);parser.add_argument('--socket');args=parser.parse_args()
    compose=['docker','compose','-f','compose.xquartz.yaml']
    try:
        if args.command=='_serve':serve(args.socket)
        elif args.command=='prepare':start_proxy();print('Authenticated XQuartz bridge ready on 127.0.0.1:6099')
        elif args.command=='build':
            subprocess.run(['docker','compose','-f','compose.sim.yaml','build'],cwd=ROOT,check=True)
            subprocess.run(['docker','compose','-f','compose.control.yaml','build'],cwd=ROOT,check=True)
            subprocess.run(compose+['build'],cwd=ROOT,check=True)
        elif args.command=='start':
            start_proxy();subprocess.run(compose+['up','-d'],cwd=ROOT,check=True)
        elif args.command=='stop':
            subprocess.run(compose+['stop','control'],cwd=ROOT,check=True)
            pid=proxy_running()
            if pid:os.kill(pid,signal.SIGTERM)
            PID.unlink(missing_ok=True)
        elif args.command=='status':
            print('XQuartz bridge:', 'running' if proxy_running() else 'stopped')
            subprocess.run(compose+['ps'],cwd=ROOT,check=True)
    except (RuntimeError,subprocess.CalledProcessError,OSError) as error:
        print(str(error),file=sys.stderr);return 1
    return 0


if __name__=='__main__':sys.exit(main())
