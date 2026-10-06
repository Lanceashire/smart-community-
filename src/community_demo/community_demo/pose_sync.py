"""Interpolate wheel samples and propagate an image-time correction forward."""
import math

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def interpolate(samples, stamp, maximum_gap=.15):
    for (ta, a), (tb, b) in zip(samples, list(samples)[1:]):
        if ta <= stamp <= tb and 0 < tb-ta <= maximum_gap+1e-9:
            k=(stamp-ta)/(tb-ta)
            return (a[0]+k*(b[0]-a[0]), a[1]+k*(b[1]-a[1]),
                    wrap(a[2]+k*wrap(b[2]-a[2])))
    return None

def field_pose(reference_field, reference_wheel, wheel):
    angle=reference_field[2]-reference_wheel[2]
    c,s=math.cos(angle),math.sin(angle)
    dx,dy=wheel[0]-reference_wheel[0],wheel[1]-reference_wheel[1]
    return (reference_field[0]+c*dx-s*dy, reference_field[1]+s*dx+c*dy,
            wrap(reference_field[2]+wrap(wheel[2]-reference_wheel[2])))
