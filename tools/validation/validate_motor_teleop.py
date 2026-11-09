#!/usr/bin/env python3
"""Wheels-off-ground brief /cmd_vel test, 15% duty, then watchdog stop."""
import json
import time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray
from sensor_msgs.msg import JointState
import math

PINS={27:(4,1),29:(0,0),31:(0,1),33:(6,1)}
def state():
    return {pin:{k:int((Path('/sys/class/pwm/pwmchip%d/pwm%d'%(chip,ch))/k).read_text()) for k in ('enable','duty_cycle')} for pin,(chip,ch) in PINS.items()}
def main():
    rclpy.init();node=rclpy.create_node('motor_teleop_validation');samples=[];joints=[];drops=[]
    node.create_subscription(Int64MultiArray,'/rdk_debug/encoder_counts',lambda m:samples.append(list(m.data)),10)
    node.create_subscription(JointState,'/joint_states',lambda m:joints.append(m),10)
    node.create_subscription(Int64MultiArray,'/rdk_debug/encoder_event_drops',lambda m:drops.append(list(m.data)),10)
    pub=node.create_publisher(Twist,'/cmd_vel',10)
    def wait(seconds):
        start=time.monotonic()
        while time.monotonic()-start<seconds:rclpy.spin_once(node,timeout_sec=.02)
    def send(v,seconds):
        start=time.monotonic();msg=Twist();msg.linear.x=v
        while time.monotonic()-start<seconds:pub.publish(msg);wait(.04)
    results={}
    try:
        ready=time.monotonic()
        while not samples and time.monotonic()-ready<8:wait(.1)
        assert samples,'No encoder telemetry'
        assert all(v['enable']==0 for v in state().values()),state()
        for name,v,active in [('forward',.225,(29,31)),('reverse',-.225,(27,33))]:
            initial=list(samples[-1]);send(v,.65);current=state()
            assert all(current[p]['enable']==1 and 0<current[p]['duty_cycle']<=1000000 for p in active),(name,current)
            assert all(current[p]['enable']==0 for p in PINS if p not in active),(name,current)
            # Publish nothing: the driver must stop using its own timeout.
            wait(.8);stopped=state();assert all(x['enable']==0 and x['duty_cycle']==0 for x in stopped.values()),stopped
            delta=[samples[-1][i]-initial[i] for i in (0,1)]
            assert all(d<0 for d in delta) if v>0 else all(d>0 for d in delta),(name,delta)
            results[name]={'raw_E1_E2_delta':delta,'active_pins':active,'watchdog_stopped':True}
            wait(1.2)
        assert drops and drops[-1]==[0],drops[-1:]
        assert joints,'No wheel joint feedback'
        positions=dict(zip(joints[-1].name,joints[-1].position))
        for i,name in enumerate(('left_wheel_joint','right_wheel_joint')):
            expected=-samples[-1][i]/2496*2*math.pi
            assert abs(positions[name]-expected)<.02,(name,positions[name],expected)
        print(json.dumps({'passed':True,'max_test_duty_percent':15,'tests':results,'gpio_event_queue_drops':drops[-1],'joint_radian_conversion_verified':True}),flush=True)
    finally:
        send(0.,.3);node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
