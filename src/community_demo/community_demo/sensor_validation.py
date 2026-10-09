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


def valid_laser_scan(m):
    """Only explicit +inf is a valid no-return; NaN/negative/over-range is invalid."""
    values=(m.angle_min,m.angle_max,m.angle_increment,m.range_min,m.range_max)
    if (stamp_ns(m.header.stamp) is None or not m.ranges or
            not all(math.isfinite(v) for v in values) or m.angle_increment <= 0 or
            not 0 < m.range_min < m.range_max or
            abs(m.angle_max-(m.angle_min+(len(m.ranges)-1)*m.angle_increment)) > 1e-4):
        return False
    if not any(abs(math.atan2(math.sin(m.angle_min+i*m.angle_increment),
                             math.cos(m.angle_min+i*m.angle_increment))) < .35
               for i in range(len(m.ranges))):
        return False
    return all(r == math.inf or (math.isfinite(r) and m.range_min <= r <= m.range_max)
               for r in m.ranges)


def decode_xyz_cloud(m):
    """Validate ROS PointCloud2 layout, then decode FLOAT32 XYZ including padded rows."""
    import numpy as np
    from sensor_msgs.msg import PointField
    if (stamp_ns(m.header.stamp) is None or m.width <= 0 or m.height <= 0 or
            m.point_step < 12 or m.row_step < m.width*m.point_step or
            len(m.data) != m.height*m.row_step):
        return None
    fields={}
    for f in m.fields:
        if f.name in ('x','y','z'):
            if f.name in fields or f.datatype != PointField.FLOAT32 or f.count != 1:
                return None
            if f.offset < 0 or f.offset+4 > m.point_step:
                return None
            fields[f.name]=f.offset
    if len(fields) != 3 or len(set(fields.values())) != 3:
        return None
    offsets=sorted(fields.values())
    if any(b-a < 4 for a,b in zip(offsets,offsets[1:])):
        return None
    try:
        dtype=np.dtype({'names':['x','y','z'],
                        'formats':[('>' if m.is_bigendian else '<')+'f4']*3,
                        'offsets':[fields[c] for c in 'xyz'],'itemsize':m.point_step})
        points=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,
                          strides=(m.row_step,m.point_step))
        xyz=np.column_stack([points[c].reshape(-1) for c in 'xyz'])
    except (ValueError,TypeError,BufferError,OverflowError):
        return None
    return xyz if np.isfinite(xyz).all(axis=1).any() else None
