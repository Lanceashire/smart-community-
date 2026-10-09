"""Known-start heading from source-stamped IMU orientation; no truth pose input."""
import math
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
