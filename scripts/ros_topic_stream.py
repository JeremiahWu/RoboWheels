#!/usr/bin/env python3
"""Read-only selected ROS topics over SSH stdio (CDR; no command topics)."""
import json
import struct
import sys
import time
import rclpy
from rclpy.serialization import serialize_message, deserialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
TOPICS = {'/cam0/image_raw','/cam1/image_raw','/cam0/camera_info','/cam1/camera_info',
          '/rdk_debug/encoder_counts','/rdk_debug/encoder_rates','/rdk_debug/encoder_invalid','/rdk_debug/encoder_event_drops',
          '/imu0','/imu_data','/stereo/points','/stereo/depth',
          '/StereoNetNode/stereonet_pointcloud2','/StereoNetNode/rectify_left_image',
          '/StereoNetNode/rectify_right_image','/odom','/joint_states','/wheel_speeds','/wheel_speed_targets','/tf','/tf_static'}
def qos(topic):
    return QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE if topic in ('/tf','/tf_static') else ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL if topic=='/tf_static' else DurabilityPolicy.VOLATILE)
def send(node):
    subs={};last={}
    def cb(topic,kind,msg):
        now=time.monotonic()
        interval=.2 if 'image_raw' in topic or 'rectify' in topic else .33 if topic in ('/stereo/points','/stereo/depth') else .03 if 'imu' in topic else 0
        if now-last.get(topic,0)<interval:return
        last[topic]=now
        header=json.dumps([topic,kind]).encode();data=serialize_message(msg)
        sys.stdout.buffer.write(struct.pack('!II',len(header),len(data))+header+data);sys.stdout.buffer.flush()
    def discover():
        for topic,types in node.get_topic_names_and_types():
            if topic in TOPICS and topic not in subs and types:
                kind=types[0]
                subs[topic]=node.create_subscription(get_message(kind),topic,lambda m,t=topic,k=kind:cb(t,k,m),qos(topic))
                print('Forwarding '+topic,file=sys.stderr,flush=True)
    node.create_timer(2,discover);discover();rclpy.spin(node)
def upright_image(msg):
    """Display-only 180-degree rotation; do not use for calibrated processing."""
    import copy
    import numpy as np
    sizes={'mono8':1,'8UC1':1,'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4,'16UC1':2,'32FC1':4}
    if msg.encoding not in sizes:raise ValueError('Unsupported display encoding '+msg.encoding)
    bpp=sizes[msg.encoding]
    pixels=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*bpp].reshape(msg.height,msg.width,bpp)
    result=copy.copy(msg);result.step=msg.width*bpp;result.data=pixels[::-1,::-1,:].copy().tobytes()
    return result

def receive(node):
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point
    markers=node.create_publisher(Marker,'/rdk_debug/imu',10)
    encoder_markers=node.create_publisher(Marker,'/rdk_debug/encoders',10)
    pubs={};count={};views={};stereo={};wheel_speeds=None;wheel_targets=None
    from sensor_msgs.msg import Image
    stereo_pub=node.create_publisher(Image,'/rdk_debug/stereo/upright',qos('/cam0/image_raw'))
    display_topics={'/cam0/image_raw':'/rdk_debug/cam0/upright','/cam1/image_raw':'/rdk_debug/cam1/upright','/stereo/depth':'/rdk_debug/depth/upright'}
    def exact(size):
        data=bytearray()
        while len(data)<size:
            chunk=sys.stdin.buffer.read(size-len(data))
            if not chunk:raise EOFError
            data.extend(chunk)
        return bytes(data)
    try:
        while True:
            h,d=struct.unpack('!II',exact(8))
            if h>4096 or d>64000000:raise ValueError('Invalid frame length')
            topic,kind=json.loads(exact(h));data=exact(d)
            if topic not in TOPICS:raise ValueError('Unapproved topic')
            if topic not in pubs:pubs[topic]=(get_message(kind),node.create_publisher(get_message(kind),topic,qos(topic)))
            cls,pub=pubs[topic];msg=deserialize_message(data,cls);pub.publish(msg)
            if topic in display_topics:
                if topic not in views:views[topic]=node.create_publisher(cls,display_topics[topic],qos(topic))
                upright=upright_image(msg)
                views[topic].publish(upright)
                if topic in ('/cam0/image_raw','/cam1/image_raw'):
                    stereo[topic]=upright
                    if len(stereo)==2:
                        left=stereo['/cam0/image_raw'];right=stereo['/cam1/image_raw']
                        delta=abs((left.header.stamp.sec-right.header.stamp.sec)+(left.header.stamp.nanosec-right.header.stamp.nanosec)*1e-9)
                        if delta<.1 and left.encoding==right.encoding=='mono8' and left.height==right.height:
                            import copy,numpy as np
                            pair=copy.copy(left);pair.width=left.width+right.width;pair.step=pair.width
                            pair.data=np.concatenate([np.frombuffer(bytes(left.data),dtype=np.uint8).reshape(left.height,left.width),np.frombuffer(bytes(right.data),dtype=np.uint8).reshape(right.height,right.width)],axis=1).tobytes()
                            stereo_pub.publish(pair);stereo.clear()
            if topic=='/wheel_speeds':wheel_speeds=list(msg.data)
            if topic=='/wheel_speed_targets':wheel_targets=list(msg.data)
            if topic=='/rdk_debug/encoder_counts':
                m=Marker();m.header.frame_id='camera_link';m.header.stamp=node.get_clock().now().to_msg()
                m.ns='encoder_ticks';m.id=0;m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD;m.lifetime.sec=1
                m.pose.orientation.w=1.;m.pose.position.x=.7;m.pose.position.z=.5;m.scale.z=.045
                m.color.r=1.;m.color.g=.8;m.color.a=1.
                m.text='Raw ticks\nB / E1: %d\nA / E2: %d'%(msg.data[0],msg.data[1])
                if wheel_speeds is not None:m.text+='\nv L/R m/s: %.3f / %.3f'%tuple(wheel_speeds[:2])
                if wheel_targets is not None:m.text+='\ntarget L/R: %.3f / %.3f'%tuple(wheel_targets[:2])
                encoder_markers.publish(m)
            if topic=='/imu0':
                m=Marker();m.header=msg.header;m.ns='acceleration';m.id=0;m.type=Marker.ARROW;m.action=Marker.ADD
                m.pose.orientation.w=1.;m.scale.x=.015;m.scale.y=.035;m.scale.z=.05
                m.color.g=1.;m.color.a=1.
                a=msg.linear_acceleration
                m.points=[Point(),Point(x=a.x*.05,y=a.y*.05,z=a.z*.05)];markers.publish(m)
                m=Marker();m.header=msg.header;m.ns='readings';m.id=1;m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD
                m.pose.orientation.w=1.;m.pose.position.z=.65;m.scale.z=.055;m.color.r=1.;m.color.g=1.;m.color.b=1.;m.color.a=1.
                g=msg.angular_velocity
                m.text='accel m/s2: %.2f %.2f %.2f\ngyro rad/s: %.3f %.3f %.3f'%(a.x,a.y,a.z,g.x,g.y,g.z)
                markers.publish(m)
            count[topic]=count.get(topic,0)+1
            if count[topic]==1:print('Received '+topic,file=sys.stderr,flush=True)
            rclpy.spin_once(node,timeout_sec=0)
    except EOFError:pass
    finally:print(json.dumps(count),file=sys.stderr,flush=True)
if __name__=='__main__':
    mode=sys.argv[1];sys.argv=sys.argv[:1];rclpy.init();n=rclpy.create_node('rdk_readonly_'+mode)
    try:(send if mode=='send' else receive)(n)
    finally:n.destroy_node();rclpy.shutdown()
