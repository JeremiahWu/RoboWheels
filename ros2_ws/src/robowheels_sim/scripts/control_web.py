#!/usr/bin/env python3
"""Local RViz dashboard backed by a real FollowJointTrajectory action client."""
import json
import math
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from trajectory_msgs.msg import JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory


class ControlWeb(Node):
    def __init__(self):
        super().__init__('control_web')
        description = Path(get_package_share_directory('robowheels_description'))
        report = json.loads((description/'config/control_model_report.json').read_text())
        self.names = [j['name'] for j in report['joints']]
        self.axes = report['joints']
        self.lock = threading.RLock()
        self.positions = {name:0. for name in self.names}
        self.last_state = 0
        self.last_odom = 0
        self.base = {'x':0.,'y':0.,'yaw':0.,'linear':0.,'angular':0.,'left_position':0.,'right_position':0.,'left_velocity':0.,'right_velocity':0.}
        self.base_publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.path_publisher = self.create_publisher(NavPath,'/base_path',1)
        self.path = NavPath();self.path.header.frame_id='odom'
        self.create_subscription(Odometry,'/odom',self.odometry,10)
        self.create_timer(.2,self.publish_path)
        self.execution = {'status':'idle','message':'等待轨迹','desired':[], 'actual':[], 'error':[]}
        self.goal_handle = None
        self.cancel_requested = False
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.create_subscription(JointState,'/joint_states',self.joint_state,10)

    def joint_state(self, msg):
        with self.lock:
            self.positions.update({name:float(position) for name,position in zip(msg.name,msg.position) if name in self.positions})
            for side in ('left','right'):
                name=f'{side}_wheel_joint'
                if name in msg.name:
                    i=msg.name.index(name)
                    if i<len(msg.position):self.base[f'{side}_position']=float(msg.position[i])
                    if i<len(msg.velocity):self.base[f'{side}_velocity']=float(msg.velocity[i])
            self.last_state = time.monotonic()

    def odometry(self,msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        with self.lock:
            self.last_odom=time.monotonic()
            self.base.update(x=p.x,y=p.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                             linear=msg.twist.twist.linear.x,angular=msg.twist.twist.angular.z)
            last=self.path.poses[-1].pose.position if self.path.poses else None
            if last is None or math.hypot(p.x-last.x,p.y-last.y)>.005:
                pose=PoseStamped();pose.header=msg.header;pose.pose=msg.pose.pose
                self.path.poses.append(pose)
                if len(self.path.poses)>2000:self.path.poses.pop(0)

    def publish_path(self):
        with self.lock:
            self.path.header.stamp=self.get_clock().now().to_msg()
            self.path_publisher.publish(self.path)

    def drive(self,data):
        linear,angular=float(data.get('linear',0)),float(data.get('angular',0))
        if not all(math.isfinite(v) for v in (linear,angular)):raise ValueError()
        command=Twist();command.linear.x=max(-.25,min(.25,linear));command.angular.z=max(-1.,min(1.,angular))
        self.base_publisher.publish(command)
        return {'ok':True,'linear':command.linear.x,'angular':command.angular.z}

    def snapshot(self):
        with self.lock:
            return {'ready':self.client.server_is_ready() and time.monotonic()-self.last_state<2,
                    'hardware':'mock_components/GenericSystem','joints':self.names,'positions':dict(self.positions),
                    'execution':dict(self.execution),'limits':[-.65,.65], 'axes':self.axes,
                    'base':dict(self.base,ready=time.monotonic()-self.last_odom<2)}

    def submit(self, data):
        with self.lock:
            if not self.client.server_is_ready() or time.monotonic()-self.last_state>2:
                return {'error':'控制器或位置反馈尚未就绪'},503
            if self.execution['status'] in ('queued','running','canceling'):
                return {'error':'已有轨迹执行中，请先取消或等待完成'},409
            current = [self.positions[name] for name in self.names]
            preset = data.get('preset')
            if preset == 'demo':
                targets = [[.12,-.12,.16,-.10,.10,-.12,.12,-.16,.10,-.10],
                           [-.12,.12,-.16,.10,-.10,.12,-.12,.16,-.10,.10],[0.]*10]
                durations = [3.,3.,3.]
            else:
                target = [0.]*len(self.names) if preset == 'home' else data.get('positions')
                if not isinstance(target,list) or len(target)!=len(self.names):
                    return {'error':'需要完整的 10 个关节位置'},400
                try:
                    target = [float(value) for value in target]
                    duration = float(data.get('duration',4))
                except (ValueError,TypeError):
                    return {'error':'关节位置和时长必须为数字'},400
                if not all(math.isfinite(v) and -.65<=v<=.65 for v in target) or not math.isfinite(duration) or not 1<=duration<=15:
                    return {'error':'位置范围为 ±0.65 rad，时长为 1–15 秒'},400
                minimum = max(abs(a-b) for a,b in zip(current,target))*6
                if duration < minimum:
                    return {'error':f'该移动请使用至少 {minimum:.1f} 秒，或减小幅度'},400
                targets,durations=[target],[duration]
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self.names
            elapsed=0.
            for target,duration in zip(targets,durations):
                elapsed+=duration
                point=JointTrajectoryPoint();point.positions=target;point.velocities=[0.]*len(self.names)
                point.time_from_start.sec=int(elapsed);point.time_from_start.nanosec=round((elapsed-int(elapsed))*1e9)
                goal.trajectory.points.append(point)
            self.goal_handle=None;self.cancel_requested=False
            self.execution={'status':'queued','message':'正在提交轨迹','desired':[], 'actual':[], 'error':[]}
            future=self.client.send_goal_async(goal,feedback_callback=self.feedback)
            future.add_done_callback(self.accepted)
            return {'ok':True,'points':len(targets),'duration':elapsed},202

    def accepted(self, future):
        try:
            handle=future.result()
            with self.lock:
                if not handle.accepted:
                    self.execution.update(status='rejected',message='控制器拒绝轨迹');return
                self.goal_handle=handle
                self.execution.update(status='running',message='轨迹执行中')
                if self.cancel_requested:
                    handle.cancel_goal_async();self.execution.update(status='canceling',message='正在取消并保持当前位置')
            handle.get_result_async().add_done_callback(self.finished)
        except Exception as error:
            with self.lock:self.execution.update(status='error',message=str(error))

    def feedback(self, msg):
        feedback=msg.feedback
        with self.lock:
            self.execution.update(desired=list(feedback.desired.positions),actual=list(feedback.actual.positions),error=list(feedback.error.positions))

    def finished(self, future):
        try:
            wrapped=future.result();result=wrapped.result
            if wrapped.status==GoalStatus.STATUS_SUCCEEDED and result.error_code==0:
                status,message='succeeded','轨迹执行成功'
            elif wrapped.status==GoalStatus.STATUS_CANCELED:
                status,message='canceled','轨迹已取消，保持当前位置'
            else:status,message='aborted',f'轨迹未完成：{result.error_string} ({result.error_code})'
            with self.lock:
                self.execution.update(status=status,message=message,error_code=result.error_code)
                self.goal_handle=None
        except Exception as error:
            with self.lock:self.execution.update(status='error',message=str(error))

    def cancel(self):
        with self.lock:
            if self.execution['status'] not in ('queued','running','canceling'):
                return {'ok':True,'message':'没有执行中的轨迹'}
            self.cancel_requested=True
            if self.goal_handle:self.goal_handle.cancel_goal_async()
            self.execution.update(status='canceling',message='正在取消并保持当前位置')
            return {'ok':True}


def main():
    rclpy.init();node=ControlWeb()
    web=Path(get_package_share_directory('robowheels_sim'))/'web'
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self,*args,**kwargs):super().__init__(*args,directory=str(web),**kwargs)
        def log_message(self,*_):pass
        def respond(self,value,status=200):
            body=json.dumps(value,allow_nan=False).encode()
            self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8')
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        def do_GET(self):
            if urlsplit(self.path).path=='/api/state':self.respond(node.snapshot());return
            if self.path=='/':self.path='/control.html'
            super().do_GET()
        def do_POST(self):
            try:
                length=int(self.headers.get('Content-Length',0))
                if not 0<length<16000 or self.headers.get_content_type()!='application/json':raise ValueError()
                data=json.loads(self.rfile.read(length))
                if not isinstance(data,dict):raise ValueError()
                if self.path=='/api/trajectory':
                    value,status=node.submit(data);self.respond(value,status)
                elif self.path=='/api/cancel':self.respond(node.cancel())
                elif self.path=='/api/cmd_vel':self.respond(node.drive(data))
                else:self.respond({'error':'Not found'},404)
            except (ValueError,TypeError):self.respond({'error':'无效的 JSON 请求'},400)
    server=ThreadingHTTPServer(('0.0.0.0',8081),Handler);server.daemon_threads=True
    threading.Thread(target=server.serve_forever,daemon=True).start()
    node.get_logger().info('RViz control dashboard listening on port 8081')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.cancel();node.drive({});server.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
