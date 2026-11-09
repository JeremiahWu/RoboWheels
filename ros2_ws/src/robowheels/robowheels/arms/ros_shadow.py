#!/usr/bin/env python3
"""Publish measured servo state and proxy RViz panel requests to the Mac owner."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError
import rclpy
from sensor_msgs.msg import JointState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='http://host.docker.internal:18765')
    parser.add_argument('--token-file', required=True)
    args = parser.parse_args()
    token = Path(args.token_file).read_text().strip()
    lock = threading.Lock()
    shared = {'state': None, 'received': 0., 'error': 'Waiting for Mac serial bridge'}

    def request(path, data=None):
        payload = None if data is None else json.dumps(data).encode()
        req = Request(args.host+path, data=payload, headers={
            'Authorization': 'Bearer '+token, 'Content-Type': 'application/json'})
        with urlopen(req, timeout=2) as response:
            return json.load(response)

    def update():
        while True:
            try:
                state = request('/state')
                with lock:
                    shared.update(state=state, received=time.monotonic(), error='')
            except Exception as exc:
                with lock:
                    shared['error'] = str(exc)
            time.sleep(.1)

    def snapshot():
        with lock:
            state = dict(shared['state'] or {'joints': [], 'armed': False, 'busy': False})
            state['bridge_fresh'] = time.monotonic()-shared['received'] < 1.5
            state['bridge_error'] = shared['error']
            return state

    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, data):
            payload = json.dumps(data).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            self.reply(200, snapshot()) if self.path == '/state' else self.reply(404, {})

        def do_POST(self):
            if self.path != '/command':
                return self.reply(404, {})
            try:
                length = int(self.headers.get('Content-Length', 0))
                if not 0 < length <= 2048:
                    raise ValueError('Invalid length')
                data = json.loads(self.rfile.read(length))
                if data.get('action') != 'disarm' and not snapshot()['bridge_fresh']:
                    raise ValueError('Host feedback is stale')
                self.reply(200, request('/command', data))
            except HTTPError as exc:
                self.reply(exc.code, json.loads(exc.read()))
            except Exception as exc:
                self.reply(503, {'error': str(exc)})

        def log_message(self, *args):
            pass

    rclpy.init()
    node = rclpy.create_node('physical_arm_feedback')
    publisher = node.create_publisher(JointState, '/joint_states', 10)

    def publish():
        state = snapshot()
        if not state['bridge_fresh']:
            return
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        # Only measured fresh arm joints are published; no fabricated zero
        # arm readings are injected for missing hardware.
        for joint in state['joints']:
            if joint['fresh'] and joint['radians'] is not None:
                msg.name.append(joint['name'])
                msg.position.append(joint['radians'])
        msg.name.extend(['left_wheel_joint', 'right_wheel_joint'])
        msg.position.extend([0., 0.])  # Base is display-only in this mode.
        publisher.publish(msg)

    node.create_timer(.1, publish)
    threading.Thread(target=update, daemon=True).start()
    threading.Thread(target=ThreadingHTTPServer(('127.0.0.1', 8082), Handler).serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
