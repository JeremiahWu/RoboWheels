#!/usr/bin/env python3
"""Validate teleop on isolated topics; never publishes /cmd_vel."""
import subprocess
import time
import json
from pathlib import Path
import yaml
import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def main():
    root=Path(__file__).resolve().parents[2]
    cfg=yaml.safe_load((root/'ros2_ws/src/robowheels/config/joy.yaml').read_text())['teleop_twist_joy_node']
    config=Path('/tmp/robowheels-teleop-test.yaml');config.write_text(yaml.safe_dump({'/**':cfg}))
    log=open('/tmp/robowheels-teleop-test.log','w')
    child=subprocess.Popen(['ros2','run','teleop_twist_joy','teleop_node','--ros-args','--params-file',str(config),'-r','__node:=robowheels_teleop_validation','-r','joy:=/rdk_debug/test_joy','-r','cmd_vel:=/rdk_debug/test_cmd'],stdout=log,stderr=log,start_new_session=True)
    rclpy.init();node=rclpy.create_node('teleop_validation_probe');values=[]
    node.create_subscription(Twist,'/rdk_debug/test_cmd',lambda m:values.append((m.linear.x,m.angular.z)),10)
    pub=node.create_publisher(Joy,'/rdk_debug/test_joy',10)
    try:
        start=time.monotonic()
        while pub.get_subscription_count()==0 and time.monotonic()-start<8:rclpy.spin_once(node,timeout_sec=.1)
        results={}
        for name,axis,value,enabled,expected in [('forward',1,1.,1,(.3,0.)),('reverse',1,-1.,1,(-.3,0.)),('right',3,-1.,1,(0.,-2.)),('trigger_ignored',2,1.,1,(0.,0.)),('RB_release',1,1.,0,(0.,0.))]:
            values.clear();msg=Joy();msg.axes=[0.,0.,-1.,0.,0.,-1.,0.,0.];msg.axes[axis]=value;msg.buttons=[0]*11;msg.buttons[5]=enabled
            start=time.monotonic()
            while time.monotonic()-start<1.5:
                msg.header.stamp=node.get_clock().now().to_msg();pub.publish(msg);rclpy.spin_once(node,timeout_sec=.05);time.sleep(.05)
            assert values, name+' received no commands'
            actual=values[-1];assert all(abs(a-b)<1e-6 for a,b in zip(actual,expected)),(name,actual,expected)
            results[name]=actual
        print(json.dumps({'passed':True,'isolated_no_motor_commands':True,'commands':results}),flush=True)
    finally:
        import os,signal
        os.killpg(child.pid,signal.SIGTERM)
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        log.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
