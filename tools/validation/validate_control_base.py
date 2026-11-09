#!/usr/bin/env python3
"""Validate closed-loop mock-wheel odometry and concurrent arm control."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output');args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('validate_control_base')
    state={};wheel={};peak={'linear':0.,'angular':0.};limited={'linear':0.,'angular':0.}
    def odometry(msg):
        state['odom']=msg
        peak['linear']=max(peak['linear'],abs(msg.twist.twist.linear.x))
        peak['angular']=max(peak['angular'],abs(msg.twist.twist.angular.z))
    def joint_state(msg):
        wheel.update(dict(zip(msg.name,msg.position)))
    def limited_command(msg):
        state['limited_received']=True
        limited['linear']=max(limited['linear'],abs(msg.twist.linear.x))
        limited['angular']=max(limited['angular'],abs(msg.twist.angular.z))
    subscriptions=[node.create_subscription(Odometry,'/odom',odometry,10),node.create_subscription(JointState,'/joint_states',joint_state,10),
                   node.create_subscription(TwistStamped,'/base_controller/cmd_vel_out',limited_command,10)]
    publisher=node.create_publisher(Twist,'/cmd_vel',10)
    buffer=Buffer();listener=TransformListener(buffer,node)
    arm=ActionClient(node,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory')
    def run(seconds,linear=None,angular=0.):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if linear is not None:
                msg=Twist();msg.linear.x=float(linear);msg.angular.z=float(angular);publisher.publish(msg)
            rclpy.spin_once(node,timeout_sec=.025)
    def wait(future,timeout=8):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        assert future.done(),'ROS action timeout'
        return future.result()
    def yaw(msg):
        q=msg.pose.pose.orientation
        return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def stopped():
        twist=state['odom'].twist.twist
        assert abs(twist.linear.x)<.005 and abs(twist.angular.z)<.005,(twist.linear.x,twist.angular.z)
    try:
        run(2);assert 'odom' in state and {'left_wheel_joint','right_wheel_joint'}<=wheel.keys()
        assert arm.wait_for_server(timeout_sec=10)
        names=[f'{side}_{joint}' for side in ('right','left') for joint in ('shoulder_roll','shoulder_pitch','elbow_pitch','wrist_pitch','wrist_roll')]
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=names
        point=JointTrajectoryPoint();point.positions=[.06]*10;point.velocities=[0.]*10;point.time_from_start.sec=3;goal.trajectory.points=[point]
        handle=wait(arm.send_goal_async(goal));assert handle.accepted;arm_result=handle.get_result_async()
        start=state['odom'];initial_wheels=[wheel['left_wheel_joint'],wheel['right_wheel_joint']]
        run(2,.12);run(1.5)
        end=state['odom'];stopped()
        heading=yaw(start);dx=end.pose.pose.position.x-start.pose.pose.position.x;dy=end.pose.pose.position.y-start.pose.pose.position.y
        forward=dx*math.cos(heading)+dy*math.sin(heading)
        left=wheel['left_wheel_joint']-initial_wheels[0];right=wheel['right_wheel_joint']-initial_wheels[1]
        encoder_distance=.0332*(left+right)/2
        assert forward>.15 and left>0 and right>0,(forward,left,right)
        assert abs(forward-encoder_distance)<.005,(forward,encoder_distance)
        result=wait(arm_result);assert result.status==4 and result.result.error_code==0
        assert max(abs(wheel[name]-.06) for name in names)<.002
        tf=buffer.lookup_transform('odom','base_footprint',rclpy.time.Time()).transform.translation
        tf_error=math.hypot(tf.x-end.pose.pose.position.x,tf.y-end.pose.pose.position.y);assert tf_error<.005
        before_yaw=yaw(end);before=[wheel['left_wheel_joint'],wheel['right_wheel_joint']]
        run(2,0.,.5);run(1.5);stopped()
        turn=math.atan2(math.sin(yaw(state['odom'])-before_yaw),math.cos(yaw(state['odom'])-before_yaw))
        delta_left=wheel['left_wheel_joint']-before[0];delta_right=wheel['right_wheel_joint']-before[1]
        assert turn>.4 and delta_left<0 and delta_right>0,(turn,delta_left,delta_right)
        run(1.2,2.,4.);run(1.5);stopped()
        # Odometry differentiates encoder positions using callback timestamps;
        # allow small scheduling jitter, but check limited commands exactly.
        assert state.get('limited_received')
        assert limited['linear']<=.250001 and limited['angular']<=1.000001,limited
        assert peak['linear']<=.26 and peak['angular']<=1.04,peak
        report={'passed':True,'closed_loop_wheel_position_feedback':True,'forward_distance_m':forward,
                'wheel_integrated_distance_m':encoder_distance,'odom_vs_wheel_error_m':abs(forward-encoder_distance),
                'positive_turn_rad':turn,'turn_left_wheel_delta_rad':delta_left,'turn_right_wheel_delta_rad':delta_right,
                'peak_measured_velocity':peak,'peak_limited_command':limited,'timeout_stopped':True,
                'odom_vs_tf_error_m':tf_error,'concurrent_arm_action_succeeded':True}
        if args.output:Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    finally:
        publisher.publish(Twist());node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
