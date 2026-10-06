"""Reject malformed sensor payloads before they refresh navigation evidence."""
import math

def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:return None
    return stamp.sec*1_000_000_000+stamp.nanosec

def valid_rgb_image(m):
    return (stamp_ns(m.header.stamp) is not None and m.encoding in ('rgb8','bgr8') and
            m.width>0 and m.height>0 and m.step>=3*m.width and len(m.data)==m.height*m.step)

def valid_odometry(m):
    p=m.pose.pose.position;q=m.pose.pose.orientation
    values=(p.x,p.y,p.z,q.x,q.y,q.z,q.w)
    return (stamp_ns(m.header.stamp) is not None and all(math.isfinite(v) for v in values) and
            abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1.)<.01)

def pan_sample(m):
    if 'camera_pan_joint' not in m.name or stamp_ns(m.header.stamp) is None:return None
    i=m.name.index('camera_pan_joint')
    if i>=len(m.position) or i>=len(m.velocity):return None
    a,v=float(m.position[i]),float(m.velocity[i])
    return (stamp_ns(m.header.stamp)/1e9,a,v) if math.isfinite(a) and math.isfinite(v) else None
