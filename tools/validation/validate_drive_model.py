#!/usr/bin/env python3
"""Validate real/simulation STEP geometry and differential-drive equations."""
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/robowheels'))
from robowheels.base.differential_drive import DifferentialDrive

def main():
    model=DifferentialDrive()
    cad=json.loads((ROOT/'tools/cad/model_config.json').read_text())
    assert abs(model.radius-cad['wheel_radius_m'])<1e-12
    robot=ET.parse(ROOT/'ros2_ws/src/robowheels_description/urdf/robot_control.urdf').getroot()
    ys=[]
    for name in ('left_wheel_joint','right_wheel_joint'):
        joint=robot.find("joint[@name='%s']"%name)
        ys.append(float(joint.find('origin').get('xyz').split()[1]))
        assert joint.find('axis').get('xyz')=='0 1 0'
    assert abs(ys[0]-ys[1]-model.separation)<1e-9
    for filename in ('ros2_ws/src/robowheels/config/motor.yaml','ros2_ws/src/robowheels_sim/config/controllers.yaml'):
        text=(ROOT/filename).read_text()
        for key,value in [('wheel_radius',model.radius),('wheel_separation',model.separation)]:
            actual=float(re.search(r'^\s*'+key+r':\s*([\d.eE+-]+)',text,re.M).group(1))
            assert abs(actual-value)<1e-12,(filename,key,actual,value)
    motor=(ROOT/'ros2_ws/src/robowheels/config/motor.yaml').read_text()
    for side in ('left','right'):
        ticks=float(re.search(r'encoder_ticks_per_revolution_'+side+r':\s*([\d.]+)',motor).group(1))
        tpm=float(re.search(r'ticks_per_meter_'+side+r':\s*([\d.]+)',motor).group(1))
        assert ticks==13*48*4
        assert abs(tpm-model.ticks_per_meter(ticks))<1e-9
        assert abs((ticks/tpm)/model.radius-2*math.pi)<1e-12
    for v,w in [(0,0),(.3,0),(-.3,0),(0,2),(.2,-.8)]:
        left,right=model.inverse(v,w);a,b=model.forward(left,right)
        assert abs(a-v)<1e-12 and abs(b-w)<1e-12
        wl,wr=model.angular_rates(v,w)
        assert abs(wl*model.radius-left)<1e-12 and abs(wr*model.radius-right)<1e-12
    limited=model.inverse(.3,2,max_wheel_speed=.3)
    assert max(abs(x) for x in limited)<=.3+1e-12
    original=model.inverse(.3,2)
    assert abs(limited[0]*original[1]-limited[1]*original[0])<1e-12
    circumference=2*math.pi*model.radius
    x,y,yaw=model.integrate((0,0,0),circumference,circumference)
    assert abs(x-circumference)<1e-12 and abs(y)<1e-12 and abs(yaw)<1e-12
    # Arc of radius 1m, yaw pi/2, has endpoint (1,1).
    angle=math.pi/2
    left=(1-model.separation/2)*angle;right=(1+model.separation/2)*angle
    x,y,yaw=model.integrate((0,0,0),left,right)
    assert abs(x-1)<1e-12 and abs(y-1)<1e-12 and abs(yaw-angle)<1e-12
    print(json.dumps({'passed':True,'wheel_radius_m':model.radius,'wheel_separation_m':model.separation,'circumference_m':circumference,'tests':'STEP/simulation/real geometry, roundtrip kinematics, curvature-preserving limits, straight and exact arc odometry'}))
if __name__=='__main__':main()
