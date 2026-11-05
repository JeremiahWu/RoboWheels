"""Create an articulated, kinematic mock-hardware model from STEP instances.

Zero is the supplied CAD assembly pose, not a measured servo zero. Hub cylinder
axes are geometry-derived; rigid-part assignments and +/- 0.65 rad demo limits
remain provisional until checked against the physical linkage and wiring.
"""
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from OCP.BRep import BRep_Builder
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.gp import gp_Trsf

from export_robot import ROOT, OUT, CFG, SOURCE, box_bounds, child, vec
from inspect_step import load_instances


def hub_axis(shape):
    surfaces = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        surface = BRepAdaptor_Surface(TopoDS.Face(exp.Current()))
        if surface.GetType() == GeomAbs_Cylinder:
            cylinder = surface.Cylinder()
            p, d = cylinder.Location(), cylinder.Axis().Direction()
            surfaces.append((cylinder.Radius(), np.array([p.X(), p.Y(), p.Z()]), np.array([d.X(), d.Y(), d.Z()])))
        exp.Next()
    if not surfaces:
        raise ValueError('Servo hub has no cylindrical face')
    radius, point, axis = max(surfaces, key=lambda item: item[0])
    lo, hi = box_bounds(shape)
    center = point + axis * np.dot((lo + hi)/2 - point, axis)
    return center, axis, radius


def main():
    digest = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    if digest != CFG['step_sha256']:
        raise RuntimeError('STEP checksum changed; review the instance mapping')
    report = json.loads((OUT/'config/conversion_report.json').read_text())
    rotation = np.array(report['cad_to_ros_rotation'])
    origin = np.array(report['cad_origin_mm'])
    doc, instances = load_instances(SOURCE)
    robot = ET.parse(OUT/'urdf/robot.urdf').getroot()
    for element in list(robot):
        if element.tag in ('gazebo', 'ros2_control') or element.get('name') in ('left_arm_link', 'right_arm_link', 'left_arm_joint', 'right_arm_joint'):
            robot.remove(element)
    control = child(robot, 'ros2_control', name='RoboWheelsVirtualHardware', type='system')
    child(child(control, 'hardware'), 'plugin').text = 'mock_components/GenericSystem'
    metadata = {'step_sha256': digest, 'zero_definition': 'supplied STEP assembly pose',
                'hardware': 'mock_components/GenericSystem', 'provisional_limits_rad': [-.65,.65],
                'limitations': 'Kinematic mock feedback; not servo calibration, torque physics or collision planning.', 'joints': [], 'groups': {}}

    def export_link(name, ids, center):
        compound = TopoDS_Compound(); builder = BRep_Builder(); builder.MakeCompound(compound)
        for i in ids:
            builder.Add(compound, instances[i][0])
        matrix = rotation / 1000; offset = -matrix @ origin - center
        transform = gp_Trsf(); transform.SetValues(*[v for row,t in zip(matrix,offset) for v in (*row,t)])
        shape = BRepBuilderAPI_Transform(compound, transform, True).Shape()
        BRepMesh_IncrementalMesh(shape, .0006, False, .35, True).Perform()
        filename = f'{name}.stl'
        writer = StlAPI_Writer(); writer.ASCIIMode = False
        if not writer.Write(shape, str(OUT/'meshes'/filename)):
            raise RuntimeError(f'Failed to export {filename}')
        link = child(robot,'link',name=name)
        visual = child(link,'visual'); child(child(visual,'geometry'),'mesh',filename=f'package://robowheels_description/meshes/{filename}')
        child(child(visual,'material',name='arm_metal'),'color',rgba='0.65 0.7 0.75 1')
        metadata['groups'][name] = {'instance_ids': ids, 'cad_zero_origin_ros_m': center.tolist()}

    export_link('shoulder_housings_link',list(range(103,107))+list(range(151,155)),np.zeros(3))
    joint = child(robot,'joint',name='shoulder_housings_joint',type='fixed')
    child(joint,'parent',link='base_link');child(joint,'child',link='shoulder_housings_link')
    for side, shift in (('right',0),('left',48)):
        axes = [(104,105),(61,62),(70,71),(74,75),(80,None)]
        segments = [[59,60,61,62,63],[64,65,66,69,70,71,72],[67,68,73,74,75,76,77],[78,79,80],list(range(81,103))]
        labels = ['shoulder_roll','shoulder_pitch','elbow_pitch','wrist_pitch','wrist_roll']
        parent, parent_center = 'base_link',np.zeros(3)
        for label, ids, (front,back) in zip(labels,segments,axes):
            front += shift
            center_mm, axis, radius = hub_axis(instances[front][0])
            if back is not None:
                back += shift
                other, other_axis, _ = hub_axis(instances[back][0])
                if abs(np.dot(axis,other_axis)) < .999:
                    raise ValueError('Opposite hub axes are not parallel')
                if np.dot(axis, center_mm-other) < 0: axis = -axis
                axis_error = np.linalg.norm(np.cross(center_mm-other,axis))
                if axis_error > .2: raise ValueError(f'Hub axes do not align: {axis_error} mm')
                center_mm = (center_mm+other)/2
            else:
                axis_error = 0.
                if axis[1]<0: axis=-axis
            center = rotation@(center_mm-origin)/1000
            axis = rotation@axis; axis /= np.linalg.norm(axis)
            name=f'{side}_{label}'; link_name=f'{name}_link'
            export_link(link_name,[i+shift for i in ids],center)
            joint = child(robot,'joint',name=name,type='revolute')
            child(joint,'parent',link=parent);child(joint,'child',link=link_name)
            child(joint,'origin',xyz=vec(center-parent_center));child(joint,'axis',xyz=vec(axis))
            child(joint,'limit',lower=-.65,upper=.65,effort=1,velocity=.5)
            controlled = child(control,'joint',name=name)
            interface = child(controlled,'command_interface',name='position')
            child(interface,'param',name='min').text='-0.65';child(interface,'param',name='max').text='0.65'
            state=child(controlled,'state_interface',name='position');child(state,'param',name='initial_value').text='0.0'
            child(controlled,'state_interface',name='velocity')
            metadata['joints'].append({'name':name,'parent':parent,'child':link_name,'hub_instances':[front,back],
                                       'axis_ros':axis.tolist(),'origin_ros_m':center.tolist(),
                                       'hub_axis_alignment_error_mm':float(axis_error),'hub_radius_mm':float(radius)})
            parent,parent_center=link_name,center
    base_control=child(robot,'ros2_control',name='RoboWheelsBaseHardware',type='system')
    hardware=child(base_control,'hardware')
    child(hardware,'plugin').text='mock_components/GenericSystem'
    child(hardware,'param',name='calculate_dynamics').text='true'
    for name in ('left_wheel_joint','right_wheel_joint'):
        controlled=child(base_control,'joint',name=name)
        child(controlled,'command_interface',name='velocity')
        state=child(controlled,'state_interface',name='position');child(state,'param',name='initial_value').text='0.0'
        velocity=child(controlled,'state_interface',name='velocity');child(velocity,'param',name='initial_value').text='0.0'
    metadata['base_hardware'] = {'name':'RoboWheelsBaseHardware','plugin':'mock_components/GenericSystem','calculate_dynamics':True,'wheel_command_interface':'velocity','wheel_state_interfaces':['position','velocity']}
    ET.indent(robot)
    ET.ElementTree(robot).write(OUT/'urdf/robot_control.urdf',encoding='utf-8',xml_declaration=True)
    (OUT/'config/control_model_report.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print('Exported articulated model:',len(metadata['joints']),'arm joints',flush=True)


if __name__=='__main__':
    main()
