"""Metric drivable corridors from the supplied 4.2 m map, and full footprint."""
import math
# left,bottom,right,top. These polygons constrain navigation; they add no walls.
CORRIDORS=[(-2,1.4,1.4,2),(-2,.2,-1.4,2),(-2,.2,.2,.8),(-.4,-2,.2,.8),(-.4,-2,1.4,-1.4),(.8,-2,1.4,2)]
HALF_LENGTH=.11
HALF_WIDTH=.1125  # includes outer wheel faces

def in_road(x,y,margin=0):
    return any(a+margin<=x<=c-margin and b+margin<=y<=d-margin for a,b,c,d in CORRIDORS)

def footprint(x,y,yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    # Sample the full body, including edges; corner-only tests fail at intersections.
    return [(x+c*u-s*v,y+s*u+c*v) for u in [-HALF_LENGTH+i*HALF_LENGTH/4 for i in range(9)] for v in [-HALF_WIDTH+i*HALF_WIDTH/4 for i in range(9)]]

def safe_pose(x,y,yaw,margin=.008):
    return all(in_road(a,b,margin) for a,b in footprint(x,y,yaw))

def predict(x,y,yaw,v,w,dt):
    if abs(w)<1e-7:return x+v*dt*math.cos(yaw),y+v*dt*math.sin(yaw),yaw
    return x+v/w*(math.sin(yaw+w*dt)-math.sin(yaw)),y-v/w*(math.cos(yaw+w*dt)-math.cos(yaw)),yaw+w*dt
