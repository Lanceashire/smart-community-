"""Known-start heading from source-stamped IMU orientation; no truth pose input."""
import math
from collections import deque
from community_demo.sensor_validation import stamp_ns
from community_demo.pose_sync import wrap

def imu_sample(message):
    q=message.orientation;w=message.angular_velocity.z
    values=(q.x,q.y,q.z,q.w,w)
    if not all(math.isfinite(x) for x in values):return None
    if message.orientation_covariance[0]<0 or not .95<=sum(x*x for x in values[:4])<=1.05:return None
    source=stamp_ns(message.header.stamp)
    if source is None:return None
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    return source/1e9,yaw,w

def heading_at(samples,source):
    if not samples:return None
    t,a,w=min(samples,key=lambda p:abs(p[0]-source))
    if abs(t-source)>.05+1e-9:return None
    return wrap(a+w*(source-t))

def map_heading_at(samples,source,offset=0.):
    heading=heading_at(samples,source)
    return None if heading is None else wrap(heading+offset)

class PaintHeadingCalibration:
    """Bounded map/IMU alignment from repeated observable forward-camera paint.

    Only the initial six seconds of valid registration may calibrate the prior.
    IMU increments remain source-timed and are never replaced by simulator yaw.
    """
    def __init__(self):
        self.offset=0.;self.start=None;self.last=None;self.samples=deque(maxlen=3)

    def update(self,stamp,innovation,improvement):
        if not all(math.isfinite(x)for x in(stamp,innovation,improvement)):return self.offset
        if self.start is None:self.start=stamp
        if self.last is not None and stamp<=self.last:return self.offset
        self.last=stamp
        if stamp-self.start>6. or improvement<=.0002:
            self.samples.clear();return self.offset
        proposed=self.offset+innovation
        if abs(proposed)>.10:self.samples.clear();return self.offset
        self.samples.append(proposed)
        if len(self.samples)==3 and max(self.samples)-min(self.samples)<=.012:
            target=sorted(self.samples)[1]
            self.offset+=max(-.012,min(.012,target-self.offset))
        return self.offset
