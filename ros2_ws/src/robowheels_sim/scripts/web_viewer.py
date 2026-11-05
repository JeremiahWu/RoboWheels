#!/usr/bin/env python3
"""Serve the local simulation viewer and a small JSON bridge to ROS 2."""
import json
import math
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


class Viewer(Node):
    def __init__(self):
        super().__init__('sim_web_viewer')
        self.lock = threading.Lock()
        self.state = {'odom': None, 'pose': None, 'joints': {}, 'sim_time': 0}
        self.last_received = 0
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_subscription(TFMessage, '/sim/ground_truth', self.poses, 10)

    def odom(self, msg):
        with self.lock:
            self.last_received = time.monotonic()
            self.state['odom'] = {'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y,
                                  'linear': msg.twist.twist.linear.x, 'angular': msg.twist.twist.angular.z}
            self.state['sim_time'] = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def poses(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'robowheels':
                p, q = transform.transform.translation, transform.transform.rotation
                with self.lock:
                    self.state['pose'] = {'position': [p.x, p.y, p.z], 'quaternion': [q.x, q.y, q.z, q.w]}
                break

    def joints(self, msg):
        with self.lock:
            self.state['joints'] = dict(zip(msg.name, msg.position))


def main():
    rclpy.init()
    node = Viewer()
    web = Path(get_package_share_directory('robowheels_sim')) / 'web'
    description = Path(get_package_share_directory('robowheels_description'))

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(web), **kwargs)

        def log_message(self, *_):
            pass

        def json_response(self, value, status=200):
            body = json.dumps(value, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def translate_path(self, path):
            relative = unquote(urlsplit(path).path)
            if relative.startswith('/robot/'):
                parts = Path(relative.removeprefix('/robot/')).parts
                # colcon symlink-install links individual files outside share/.
                # Accept only package asset basenames, without traversal.
                if len(parts) == 2 and parts[0] in ('urdf', 'meshes') and parts[1] not in ('.', '..'):
                    return str(description.joinpath(*parts))
                return str(web / '__not_found__')
            return super().translate_path(path)

        def do_GET(self):
            if urlsplit(self.path).path == '/api/state':
                with node.lock:
                    state = dict(node.state, connected=time.monotonic() - node.last_received < 2)
                self.json_response(state)
            else:
                super().do_GET()

        def do_POST(self):
            if self.path != '/api/cmd_vel':
                self.json_response({'error': 'Not found'}, 404)
                return
            try:
                length = int(self.headers.get('Content-Length', 0))
                if not 0 < length < 1024 or self.headers.get_content_type() != 'application/json':
                    raise ValueError('Expected a small JSON command')
                data = json.loads(self.rfile.read(length))
                linear, angular = float(data.get('linear', 0)), float(data.get('angular', 0))
                if not all(math.isfinite(v) for v in (linear, angular)):
                    raise ValueError('Non-finite command')
                command = Twist()
                command.linear.x = max(-.25, min(.25, linear))
                command.angular.z = max(-1., min(1., angular))
                node.publisher.publish(command)
                self.json_response({'ok': True})
            except (ValueError, TypeError, json.JSONDecodeError):
                self.json_response({'error': 'Invalid command'}, 400)

    server = ThreadingHTTPServer(('0.0.0.0', 8080), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info('Simulation webpage listening on port 8080')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
