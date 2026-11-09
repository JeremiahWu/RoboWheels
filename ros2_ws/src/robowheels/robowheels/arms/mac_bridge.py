#!/usr/bin/env python3
"""Mac USB servo owner and authenticated local HTTP API for the ROS viewer."""
import argparse
import hmac
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time

from robowheels.arms.desheng_protocol import packet, query


class ServoBridge:
    def __init__(self, config, zero, port):
        self.config, self.port = config, port
        self.lock = threading.RLock()
        self.armed = False
        self.busy_until = 0.
        self.active_id = None
        self.fault = ''
        self.items = {}
        for joint in config['joints']:
            sid = joint['id']
            key = f'right_joint_{sid}' if sid <= 4 else f'left_joint_{sid-4}'
            offset = zero['joints'][key]
            if offset['servo_id'] != sid:
                raise ValueError('Zero mapping mismatch')
            if type(offset.get('zero_raw')) is not int or not 0 <= offset['zero_raw'] <= 4095:
                raise ValueError(f'ID {sid}: calibrate a local zero_raw before starting')
            self.items[sid] = dict(joint, zero_raw=offset['zero_raw'], raw=None,
                                   stamp=0., fault='', lo=None, hi=None, torque=None)

    def read(self, sid, addr, n):
        frame = query(self.port, sid, 2, bytes([addr, n]))
        if frame is None or len(frame) != n + 6 or frame[4] != 0:
            raise ValueError(f'ID {sid}: invalid/missing read at {addr:#x}')
        return int.from_bytes(frame[5:-1], 'big')

    def initialize(self):
        # Read-only preflight. No target writes, torque changes, or homing.
        with self.lock:
            for sid, item in self.items.items():
                try:
                    if self.read(sid, 5, 1) != sid:
                        raise ValueError('Identity mismatch')
                    item['lo'] = self.read(sid, 9, 2)
                    item['hi'] = self.read(sid, 11, 2)
                    item['torque'] = self.read(sid, 0x28, 1)
                    item['raw'] = self.read(sid, 0x38, 2)
                    item['stamp'] = time.monotonic()
                except Exception as exc:
                    item['fault'] = str(exc)

    def poll(self):
        while True:
            for sid in self.items:
                with self.lock:
                    item = self.items[sid]
                    try:
                        value = self.read(sid, 0x38, 2)
                        if item['raw'] is not None and abs(value - item['raw']) > 100:
                            item['fault'] = 'Unexpected feedback jump; restart required'
                            self.armed = False
                        item['raw'], item['stamp'] = value, time.monotonic()
                        if self.active_id == sid:
                            if not self.active_start-20 <= value <= self.active_target+20:
                                # Bounds are sorted when the command is issued.
                                item['fault'] = 'Unexpected movement; outputs disarmed'
                                self.armed = False
                    except Exception as exc:
                        self.armed = False
                        item['fault'] = str(exc)
                time.sleep(.005)
            with self.lock:
                if self.active_id is not None and time.monotonic() >= self.busy_until:
                    item = self.items[self.active_id]
                    if abs(item['raw'] - self.target_raw) > 20:
                        item['fault'] = 'Target not reached; outputs disarmed'
                        self.armed = False
                    self.active_id = None

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            joints = []
            for item in self.items.values():
                entry = dict(item)
                entry['age'] = now - item['stamp']
                entry['fresh'] = entry['age'] < self.config['stale_seconds']
                entry['relative_raw'] = None if item['raw'] is None else item['raw'] - item['zero_raw']
                entry['radians'] = None if entry['relative_raw'] is None else entry['relative_raw'] * item['sign'] * self.config['radians_per_raw']
                entry.pop('stamp')
                joints.append(entry)
            return {'armed': self.armed, 'busy': self.active_id is not None,
                    'active_id': self.active_id, 'joints': joints,
                    'scale_note': self.config['angle_scale_note'],
                    'direction_calibrated': False, 'message': self.fault}

    def command(self, data):
        with self.lock:
            action = data.get('action')
            if action == 'disarm':
                self.armed = False
                # Interrupt only the one active, trusted joint using its current readback.
                if self.active_id is not None:
                    sid = self.active_id
                    item = self.items[sid]
                    current = self.read(sid, 0x38, 2)
                    if item['fault'] or abs(current-item['raw']) > 20:
                        raise ValueError('Cannot safely issue hold; support arm and remove DC if needed')
                    self.write_position(sid, current, 300)
                    self.active_id = None
                return {'ok': True}
            if action == 'arm':
                # Only explicitly enabled joints may receive jog commands.
                for item in self.items.values():
                    if item['enabled'] and (item['fault'] or item['raw'] is None or
                            time.monotonic()-item['stamp'] > self.config['stale_seconds']):
                        raise ValueError('Enabled joint feedback is not ready')
                self.armed = True
                return {'ok': True}
            if action != 'jog':
                raise ValueError('Unsupported action')
            sid, delta = data.get('id'), data.get('delta_raw')
            if type(sid) is not int or type(delta) is not int:
                raise ValueError('Integer ID and step required')
            if not self.armed or self.active_id is not None:
                raise ValueError('Outputs disarmed or another joint moving')
            if sid not in self.items or not self.items[sid]['enabled']:
                raise ValueError('Joint is locked')
            if not 0 < abs(delta) <= self.items[sid].get('max_step_raw', self.config['max_step_raw']):
                raise ValueError('Step exceeds allowed size')
            item = self.items[sid]
            if item['fault'] or time.monotonic()-item['stamp'] > self.config['stale_seconds']:
                raise ValueError('Joint feedback is stale or faulted')
            current = self.read(sid, 0x38, 2)
            if abs(current-item['raw']) > 20:
                self.armed = False
                raise ValueError('Position changed unexpectedly')
            target = current + delta
            if abs(target-item['zero_raw']) > item.get('max_offset_raw', self.config['max_offset_raw']):
                raise ValueError('Target exceeds provisional software envelope around zero')
            if not (0 <= item['lo'] < item['hi'] <= 4095 and item['lo']+20 <= target <= item['hi']-20):
                raise ValueError('Target near device limit')
            if self.read(sid, 0x28, 1) == 0:
                raise ValueError('Torque is disabled; not enabling automatically')
            self.write_position(sid, target, self.config['duration_ms'])
            item['raw'], item['stamp'] = current, time.monotonic()
            self.active_id = sid
            self.target_raw = target
            self.active_start, self.active_target = sorted((current, target))
            self.busy_until = time.monotonic() + self.config['duration_ms']/1000 + 1
            return {'ok': True, 'id': sid, 'start_raw': current, 'target_raw': target}

    def write_position(self, sid, target, duration):
        self.port.reset_input_buffer()
        self.port.write(packet(sid, 3, bytes([0x2a])+target.to_bytes(2, 'big')+duration.to_bytes(2, 'big')))
        self.port.flush()
        self.port.read(64)


def main():
    import serial
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--zeros', required=True)
    parser.add_argument('--token-file', required=True)
    parser.add_argument('--listen', default='127.0.0.1')
    parser.add_argument('--http-port', type=int, default=18765)
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    token = Path(args.token_file).read_text().strip()
    if len(token) < 32:
        raise ValueError('Authentication token too short')
    port = serial.Serial(config['port'], config['baud'], timeout=.04, write_timeout=1, exclusive=True)
    port.dtr = port.rts = False
    bridge = ServoBridge(config, json.loads(Path(args.zeros).read_text()), port)
    bridge.initialize()
    threading.Thread(target=bridge.poll, daemon=True).start()

    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, data):
            payload = json.dumps(data, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def authenticate(self):
            return hmac.compare_digest(self.headers.get('Authorization', ''), 'Bearer '+token)

        def do_GET(self):
            if not self.authenticate():
                return self.reply(403, {'error': 'Unauthorized'})
            if self.path != '/state':
                return self.reply(404, {'error': 'Not found'})
            self.reply(200, bridge.snapshot())

        def do_POST(self):
            if not self.authenticate():
                return self.reply(403, {'error': 'Unauthorized'})
            if self.path != '/command':
                return self.reply(404, {'error': 'Not found'})
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 2048:
                    raise ValueError('Invalid request length')
                result = bridge.command(json.loads(self.rfile.read(length)))
                self.reply(200, result)
            except (ValueError, KeyError, TypeError) as exc:
                self.reply(409, {'error': str(exc)})
            except Exception as exc:
                bridge.armed = False
                self.reply(503, {'error': str(exc)})

        def log_message(self, *args):
            pass

    print(f'Servo bridge listening on {args.listen}:{args.http_port}; outputs DISARMED', flush=True)
    try:
        ThreadingHTTPServer((args.listen, args.http_port), Handler).serve_forever()
    finally:
        port.close()


if __name__ == '__main__':
    main()
