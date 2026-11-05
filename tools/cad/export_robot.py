"""Rebuild simulation meshes and URDF from the checked-in STEP assembly.

Requires cadquery-ocp 8 and numpy. Instance IDs refer to this exact STEP file;
the checksum prevents silently applying the mapping to a changed assembly.
"""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from OCP.BRep import BRep_Builder
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.StlAPI import StlAPI_Writer
from OCP.TopoDS import TopoDS_Compound
from OCP.gp import gp_Trsf, gp_Vec
from inspect_step import load_instances

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'ros2_ws/src/robowheels_description'
CFG = json.loads((ROOT / 'tools/cad/model_config.json').read_text())
SOURCE = ROOT / 'step/robot.STEP'


def vec(values):
    return ' '.join(f'{v:.9g}' for v in values)


def child(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def box_bounds(shape):
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box)
    a, b = box.CornerMin(), box.CornerMax()
    return np.array([a.X(), a.Y(), a.Z()]), np.array([b.X(), b.Y(), b.Z()])


def main():
    digest = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    if digest != CFG['step_sha256']:
        raise RuntimeError('STEP changed: inspect the assembly and update model_config.json first')
    doc, instances = load_instances(SOURCE)
    # Extracted cylinder axes and the largest chassis plate normal (see axes.py).
    up = np.array(CFG['cad_up_axis']); up /= np.linalg.norm(up)
    left = np.array(CFG['cad_left_axis']); left -= up * np.dot(left, up); left /= np.linalg.norm(left)
    forward = np.cross(left, up)
    rotation = np.array([forward, left, up])
    wheel_centers = []
    for index, axis_point in zip((11, 8), CFG['wheel_axis_points_mm']):
        lo, hi = box_bounds(instances[index][0])
        point = np.array(axis_point)
        wheel_centers.append(point + left * np.dot((lo + hi) / 2 - point, left))
    origin = sum(wheel_centers) / 2
    radius = CFG['wheel_radius_m']
    separation = float(np.linalg.norm(wheel_centers[0] - wheel_centers[1]) / 1000)
    robot = ET.Element('robot', name='robowheels')
    child(robot, 'link', name='base_footprint')
    joint = child(robot, 'joint', name='base_footprint_joint', type='fixed')
    child(joint, 'parent', link='base_footprint'); child(joint, 'child', link='base_link')
    child(joint, 'origin', xyz=vec([0, 0, radius]))
    report = {'source_sha256': digest, 'cad_origin_mm': origin.tolist(), 'cad_to_ros_rotation': rotation.tolist(),
              'wheel_radius_m': radius, 'wheel_separation_m': separation,
              'note': 'Masses, inertias and friction are estimates. Arms fixed. Legacy lidar and Raspberry Pi omitted.', 'groups': {}}
    groups = {
        'base_link': [i for i in range(34) if i not in (2, 8, 11)] + list(range(37, 46)),
        'mast_link': [46], 'arm_mount_link': list(range(47, 59)),
        'right_arm_link': list(range(59, 107)), 'left_arm_link': list(range(107, 155)),
        'astra_link': [34, 35, 36], 'left_wheel_link': [11], 'right_wheel_link': [8],
        'caster_link': [2],
    }
    OUT.joinpath('meshes').mkdir(exist_ok=True)
    for name, ids in groups.items():
        compound = TopoDS_Compound(); builder = BRep_Builder(); builder.MakeCompound(compound)
        for index in ids:
            builder.Add(compound, instances[index][0])
        trsf = gp_Trsf()
        matrix = rotation / 1000
        offset = -matrix @ origin
        trsf.SetValues(*[v for row, t in zip(matrix, offset) for v in (*row, t)])
        world_shape = BRepBuilderAPI_Transform(compound, trsf, True).Shape()
        low, high = box_bounds(world_shape)
        center, size = (low + high) / 2, high - low
        is_wheel = name in ('left_wheel_link', 'right_wheel_link')
        if is_wheel:
            center = rotation @ (wheel_centers[0 if name.startswith('left') else 1] - origin) / 1000
            center[0] = center[2] = 0
        if name == 'base_link':
            center = np.zeros(3)
        local = gp_Trsf(); local.SetTranslationPart(gp_Vec(*(-center)))
        shape = BRepBuilderAPI_Transform(world_shape, local, True).Shape()
        BRepMesh_IncrementalMesh(shape, CFG['mesh_deflection_m'], False, 0.35, True).Perform()
        writer = StlAPI_Writer(); writer.ASCIIMode = False
        if not writer.Write(shape, str(OUT / 'meshes' / f'{name}.stl')):
            raise RuntimeError(f'STL export failed: {name}')
        link = child(robot, 'link', name=name)
        visual = child(link, 'visual')
        child(child(visual, 'geometry'), 'mesh', filename=f'package://robowheels_description/meshes/{name}.stl')
        material = child(visual, 'material', name=f'{name}_material')
        child(material, 'color', rgba='0.12 0.12 0.14 1' if is_wheel else '0.65 0.7 0.75 1')
        mass = CFG['masses_kg'][name]
        inertial = child(link, 'inertial')
        com = (low + high) / 2 - center
        child(inertial, 'origin', xyz=vec(com)); child(inertial, 'mass', value=mass)
        inertia = mass / 12 * np.array([size[1]**2 + size[2]**2, size[0]**2 + size[2]**2, size[0]**2 + size[1]**2])
        child(inertial, 'inertia', ixx=inertia[0], iyy=inertia[1], izz=inertia[2], ixy=0, ixz=0, iyz=0)
        collision = child(link, 'collision', name=f'{name}_collision')
        if is_wheel:
            child(collision, 'origin', rpy=f'{math.pi/2} 0 0')
            child(child(collision, 'geometry'), 'cylinder', radius=radius, length=CFG['wheel_width_m'])
        elif name == 'caster_link':
            # Low friction ball approximation; preserves STEP visual placement.
            correction = np.array([0, 0, -radius + CFG['caster_radius_m']]) - center
            correction[:2] = 0
            child(collision, 'origin', xyz=vec(correction))
            child(child(collision, 'geometry'), 'sphere', radius=CFG['caster_radius_m'])
        else:
            child(collision, 'origin', xyz=vec(com))
            child(child(collision, 'geometry'), 'box', size=vec(size))
        if name != 'base_link':
            joint = child(robot, 'joint', name=name.replace('_link', '_joint'), type='continuous' if is_wheel else 'fixed')
            child(joint, 'parent', link='base_link'); child(joint, 'child', link=name)
            child(joint, 'origin', xyz=vec(center))
            if is_wheel:
                child(joint, 'axis', xyz='0 1 0'); child(joint, 'limit', effort=2, velocity=20)
                child(joint, 'dynamics', damping=0.01, friction=0)
        gz = child(robot, 'gazebo', reference=name)
        child(gz, 'mu1').text = str(0.01 if name == 'caster_link' else CFG['friction'])
        child(gz, 'mu2').text = str(0.01 if name == 'caster_link' else CFG['friction'])
        report['groups'][name] = {'instance_ids': ids, 'origin_m': center.tolist(), 'bbox_size_m': size.tolist(), 'mass_kg_estimate': mass}
        print(name, 'size', size.round(4).tolist(), flush=True)
    gz = child(robot, 'gazebo')
    drive = child(gz, 'plugin', filename='libignition-gazebo-diff-drive-system.so', name='ignition::gazebo::systems::DiffDrive')
    for tag, value in {'left_joint': 'left_wheel_joint', 'right_joint': 'right_wheel_joint', 'wheel_separation': separation,
                       'wheel_radius': radius, 'topic': '/sim/cmd_vel', 'odom_topic': '/odom', 'tf_topic': '/tf',
                       'frame_id': 'odom', 'child_frame_id': 'base_footprint', 'odom_publish_frequency': 50,
                       'max_linear_velocity': 0.5, 'min_linear_velocity': -0.5,
                       'max_angular_velocity': 2.0, 'min_angular_velocity': -2.0}.items():
        child(drive, tag).text = str(value)
    states = child(gz, 'plugin', filename='libignition-gazebo-joint-state-publisher-system.so', name='ignition::gazebo::systems::JointStatePublisher')
    child(states, 'topic').text = '/joint_states'
    child(states, 'joint_name').text = 'left_wheel_joint'; child(states, 'joint_name').text = 'right_wheel_joint'
    poses = child(gz, 'plugin', filename='libignition-gazebo-pose-publisher-system.so', name='ignition::gazebo::systems::PosePublisher')
    for tag, value in {'publish_model_pose': 'true', 'publish_nested_model_pose': 'true', 'publish_link_pose': 'false', 'use_pose_vector_msg': 'true', 'update_frequency': '30'}.items():
        child(poses, tag).text = value
    ET.indent(robot)
    ET.ElementTree(robot).write(OUT / 'urdf/robot.urdf', encoding='utf-8', xml_declaration=True)
    (OUT / 'config/conversion_report.json').write_text(json.dumps(report, indent=2) + '\n')
    local_reports = ROOT / '.local/cad'
    local_reports.mkdir(parents=True, exist_ok=True)
    (local_reports / 'assembly_inventory.json').write_text(json.dumps([p for _, p in instances], ensure_ascii=False, indent=2) + '\n')
    print('Export complete; wheel separation:', separation, flush=True)


if __name__ == '__main__':
    main()
