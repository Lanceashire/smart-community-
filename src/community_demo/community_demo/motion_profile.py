"""Slow before upcoming bends instead of entering them at straight-line speed."""
import math

def bend_limit(points,nearest,heading_error,distance,base):
    limit=base
    if abs(heading_error)>.18:limit=min(limit,.12)
    # Inspect upcoming geometry within 24 cm of the current path location.
    origin=points[nearest]
    for i in range(nearest+1,len(points)-1):
        if math.hypot(points[i][0]-origin[0],points[i][1]-origin[1])>.24:break
        a=points[max(nearest,i-3)];b=points[i];c=points[min(len(points)-1,i+3)]
        if math.hypot(b[0]-a[0],b[1]-a[1])<.025 or math.hypot(c[0]-b[0],c[1]-b[1])<.025:continue
        u=math.atan2(b[1]-a[1],b[0]-a[0]);v=math.atan2(c[1]-b[1],c[0]-b[0])
        if abs(math.atan2(math.sin(v-u),math.cos(v-u)))>.35:limit=min(limit,.12)
    return min(limit,.12) if distance<.12 else limit
