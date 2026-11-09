#!/usr/bin/env python3
"""Validate mock control actions, joint feedback and independently computed FK.

Run in the control container after launch. This executes small arm trajectories.
"""
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory


def main():
    rclpy.init();node=rclpy.create_node('validate_virtual_control')
    desc=Path(get_package_share_directory('robowheels_description'))
    report=json.loads((desc/'config/control_model_report.json').read_text())
    names=[j['name'] for j in report['joints']]
    model=ET.parse(desc/'urdf/robot_control.urdf').getroot()
    joints={j.find('child').get('link'):j for j in model.findall('joint')}
    latest={};samples=[]
    def receive(msg):
        latest.update(dict(zip(msg.name,msg.position)))
        samples.append([latest.get(name,0) for name in names])
    subscription=node.create_subscription(JointState,'/joint_states',receive,10)
    buffer=Buffer();listener=TransformListener(buffer,node)
    client=ActionClient(node,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory')

    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def wait(future,timeout=12):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        assert future.done(),'Timed out waiting for ROS action'
        return future.result()
    def send(target,duration):
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=names
        point=JointTrajectoryPoint();point.positions=target;point.velocities=[0.]*len(names);point.time_from_start.sec=duration
        goal.trajectory.points=[point]
        handle=wait(client.send_goal_async(goal));assert handle.accepted,'Goal rejected'
        return handle
    def rotation(axis,angle):
        axis=np.array(axis);axis/=np.linalg.norm(axis)
        x,y,z=axis;cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        return np.eye(3)+math.sin(angle)*cross+(1-math.cos(angle))*(cross@cross)
    def forward(link):
        if link=='base_footprint':return np.eye(4)
        joint=joints[link];local=np.eye(4);origin=joint.find('origin')
        if origin is not None:
            local[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            assert origin.get('rpy','0 0 0')=='0 0 0','Extend FK validator for nonzero origin rotations'
        axis=joint.find('axis')
        if axis is not None:
            local[:3,:3]=rotation(np.fromstring(axis.get('xyz'),sep=' '),latest.get(joint.get('name'),0))
        return forward(joint.find('parent').get('link'))@local
    def check_tf():
        errors=[]
        for side in ('right','left'):
            link=f'{side}_wrist_roll_link'
            transform=buffer.lookup_transform('base_footprint',link,rclpy.time.Time()).transform
            expected=forward(link);p=transform.translation;q=transform.rotation
            actual=np.array([p.x,p.y,p.z]);position_error=np.linalg.norm(actual-expected[:3,3])
            x,y,z,w=q.x,q.y,q.z,q.w
            actual_rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                                      [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                                      [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            orientation_error=np.max(np.abs(actual_rotation-expected[:3,:3]))
            assert position_error<2e-5 and orientation_error<2e-5,(side,position_error,orientation_error)
            errors.append({'side':side,'position_error_m':float(position_error),'rotation_matrix_error':float(orientation_error)})
        return errors
    try:
        assert client.wait_for_server(timeout_sec=30),'Trajectory action not available'
        spin(1);assert set(names)<=latest.keys(),'Missing joint states'
        zero=wait(send([0.]*10,2).get_result_async());assert zero.status==4 and zero.result.error_code==0
        spin(.2);zero_tf=check_tf()
        target=[.08,-.10,.12,-.06,.09,-.08,.10,-.12,.06,-.09]
        samples.clear();completed=wait(send(target,3).get_result_async())
        assert completed.status==4 and completed.result.error_code==0
        spin(.3);error=max(abs(latest[n]-v) for n,v in zip(names,target));assert error<.002,error
        assert any(.015<abs(sample[0])<.065 for sample in samples),'No interpolated positions observed'
        target_tf=check_tf()
        handle=send([-v for v in target],8);result_future=handle.get_result_async();spin(.7)
        canceled=wait(handle.cancel_goal_async());assert canceled.goals_canceling
        result=wait(result_future);assert result.status==5,result.status
        spin(.2);held=[latest[n] for n in names];spin(.5)
        hold_error=max(abs(latest[n]-value) for n,value in zip(names,held));assert hold_error<.002,hold_error
        home=wait(send([0.]*10,3).get_result_async());assert home.status==4
        spin(.2)
        print(json.dumps({'passed':True,'hardware':'mock_components/GenericSystem','joint_count':len(names),
                          'trajectory_result':completed.result.error_code,'max_position_error_rad':error,
                          'interpolation_observed':True,'cancel_status':result.status,'cancel_hold_error_rad':hold_error,
                          'zero_fk_vs_tf':zero_tf,'target_fk_vs_tf':target_tf},indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
