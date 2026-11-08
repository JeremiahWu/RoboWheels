"""Planar differential-drive kinematics shared by real-base control and tests.

REP-style convention: +x forward, +y left, +yaw counterclockwise.
Wheel rates/angles are positive when their wheels drive the vehicle forward.
"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class DifferentialDrive:
    radius: float = 0.0332
    separation: float = 0.15631239107433037

    def __post_init__(self):
        if not all(math.isfinite(x) and x > 0 for x in (self.radius,self.separation)):
            raise ValueError('Wheel radius and separation must be finite and positive')

    def inverse(self,linear,yaw_rate,max_wheel_speed=None):
        if not all(math.isfinite(x) for x in (linear,yaw_rate)):
            raise ValueError('Non-finite body command')
        left=linear-yaw_rate*self.separation/2
        right=linear+yaw_rate*self.separation/2
        if max_wheel_speed is not None:
            if not math.isfinite(max_wheel_speed) or max_wheel_speed<=0:
                raise ValueError('Wheel speed limit must be positive')
            scale=min(1.,max_wheel_speed/max(abs(left),abs(right),1e-12))
            left*=scale;right*=scale
        return left,right

    def forward(self,left,right):
        return (left+right)/2,(right-left)/self.separation

    def angular_rates(self,linear,yaw_rate):
        left,right=self.inverse(linear,yaw_rate)
        return left/self.radius,right/self.radius

    def integrate(self,pose,left_distance,right_distance):
        x,y,yaw=pose
        distance=(left_distance+right_distance)/2
        angle=(right_distance-left_distance)/self.separation
        half=angle/2
        sinc=math.sin(half)/half if abs(half)>1e-8 else 1-half*half/6
        x+=distance*sinc*math.cos(yaw+half)
        y+=distance*sinc*math.sin(yaw+half)
        yaw=math.atan2(math.sin(yaw+angle),math.cos(yaw+angle))
        return x,y,yaw

    def ticks_per_meter(self,ticks_per_revolution):
        if not math.isfinite(ticks_per_revolution) or ticks_per_revolution<=0:
            raise ValueError('Encoder resolution must be positive')
        return ticks_per_revolution/(2*math.pi*self.radius)
