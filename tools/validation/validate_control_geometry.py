#!/usr/bin/env python3
"""Compare articulated zero-pose meshes against the original STEP-derived arms."""
import json
import struct
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parents[2]
description=root/'ros2_ws/src/robowheels_description'
report=json.loads((description/'config/control_model_report.json').read_text())
original_report=json.loads((description/'config/conversion_report.json').read_text())
dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])


def vertices(name):
    raw=(description/'meshes'/f'{name}.stl').read_bytes()
    count=struct.unpack_from('<I',raw,80)[0]
    assert len(raw)==84+count*50
    return np.frombuffer(raw,dtype=dtype,offset=84,count=count)['vertices'].reshape(-1,3).astype(float)


errors={}
for side in ('right','left'):
    original=vertices(f'{side}_arm_link')+original_report['groups'][f'{side}_arm_link']['origin_m']
    groups=[vertices(name)+group['cad_zero_origin_ros_m'] for name,group in report['groups'].items() if name.startswith(side+'_')]
    housings=vertices('shoulder_housings_link')
    groups.append(housings[housings[:,1]<0] if side=='right' else housings[housings[:,1]>0])
    restored=np.concatenate(groups)
    error=float(max(np.max(np.abs(original.min(0)-restored.min(0))),np.max(np.abs(original.max(0)-restored.max(0)))))
    assert error<1e-6,(side,error)
    errors[side]=error
all_ids=[index for group in report['groups'].values() for index in group['instance_ids']]
assert sorted(all_ids)==list(range(59,155)) and len(set(all_ids))==96
assert all(abs(np.linalg.norm(joint['axis_ros'])-1)<1e-8 for joint in report['joints'])
max_error=max(joint['hub_axis_alignment_error_mm'] for joint in report['joints'])
assert max_error<.2
result={'cad_zero_assembly_bbox_error_m':errors,'all_arm_instances_assigned_once':True,'axes_normalized':True,'max_hub_axis_alignment_error_mm':max_error}
print(json.dumps(result,indent=2))
