#!/usr/bin/env python3
"""Snapshot stationary raw E1/E2 ticks for manual wheel-turn calibration."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from std_msgs.msg import Int64MultiArray

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('stage',choices=['begin','finish']);p.add_argument('wheel',choices=['left','right'])
    p.add_argument('--turns',type=int,default=5)
    p.add_argument('--allow-other-wheel',action='store_true',help='Only for a deliberate two-wheel verification trial')
    a=p.parse_args()
    if a.turns<=0:p.error('turns must be positive')
    for chip,ch in [(4,1),(0,0),(0,1),(6,1)]:
        pwm=Path('/sys/class/pwm/pwmchip%d/pwm%d'%(chip,ch))
        if pwm.exists() and int((pwm/'enable').read_text()):raise RuntimeError('Motor PWM is still enabled')
    rclpy.init();n=rclpy.create_node('encoder_manual_calibration');values=[];invalid=[];drops=[]
    n.create_subscription(Int64MultiArray,'/rdk_debug/encoder_counts',lambda m:values.append(list(m.data)),10)
    n.create_subscription(Int64MultiArray,'/rdk_debug/encoder_invalid',lambda m:invalid.append(list(m.data)),10)
    n.create_subscription(Int64MultiArray,'/rdk_debug/encoder_event_drops',lambda m:drops.append(list(m.data)),10)
    t=time.monotonic()
    while time.monotonic()-t<4:
        rclpy.spin_once(n,timeout_sec=.1)
        if len(values)>=12 and invalid:break
    n.destroy_node();rclpy.shutdown()
    if len(values)<5 or any(v!=values[-1] for v in values[-5:]):raise RuntimeError('No stable encoder sample; stop wheels and retry')
    output=Path(__file__).resolve().parents[3]/'.local/validation/manual_encoder_turns.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    data=json.loads(output.read_text()) if output.exists() else {}
    now=values[-1]
    if a.stage=='begin':
        if a.wheel in data and 'end_E1_E2' in data[a.wheel]:
            data.setdefault('history',[]).append({'wheel':a.wheel,**data[a.wheel]})
        data[a.wheel]={'begin_E1_E2':now,'turns':a.turns,'begin_invalid':invalid[-1] if invalid else None,'begin_drops':drops[-1] if drops else None};result={'wheel':a.wheel,'stage':'begin','E1_E2':now,'turns':a.turns}
    else:
        entry=data[a.wheel];delta=[now[i]-entry['begin_E1_E2'][i] for i in (0,1)];index=0 if a.wheel=='left' else 1
        if not a.allow_other_wheel and abs(delta[1-index])>4:raise RuntimeError('Other wheel also moved: '+str(delta))
        if delta[index]==0:raise RuntimeError('No wheel counts observed')
        circumference=math.pi*.0664;per_rev=abs(delta[index])/entry['turns']
        entry.update(end_drops=drops[-1] if drops else None,end_invalid=invalid[-1] if invalid else None,end_E1_E2=now,delta_E1_E2=delta,ticks_per_revolution=per_rev,ticks_per_meter=per_rev/circumference,wheel_radius_m=.0332,raw_forward_sign=1 if delta[index]>0 else -1)
        result={'wheel':a.wheel,'stage':'finish',**entry}
    output.write_text(json.dumps(data,ensure_ascii=False,indent=2));print(json.dumps(result),flush=True)
if __name__=='__main__':main()
