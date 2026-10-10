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

def body_corners(x,y,yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return [(x+c*u-s*v,y+s*u+c*v) for u,v in
            ((-HALF_LENGTH,-HALF_WIDTH),(HALF_LENGTH,-HALF_WIDTH),
             (HALF_LENGTH,HALF_WIDTH),(-HALF_LENGTH,HALF_WIDTH))]


def clip_axis(polygon,axis,bound,greater):
    """Clip a convex polygon to a closed axis-aligned half plane."""
    output=[]
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        ain=a[axis]>=bound if greater else a[axis]<=bound
        bin=b[axis]>=bound if greater else b[axis]<=bound
        if ain:output.append(a)
        if ain!=bin:
            k=(bound-a[axis])/(b[axis]-a[axis])
            output.append((a[0]+k*(b[0]-a[0]),a[1]+k*(b[1]-a[1])))
    return output


def polygon_in_road(polygon,margin=.008):
    """Exact containment in the rectangle union, including concave junctions.

    Within each vertical strip the union's Y intervals are constant. A convex
    footprint is covered iff its entire clipped Y range fits a merged interval.
    Checking only 81 body points misses small triangles at inside corners.
    """
    if not polygon or not all(math.isfinite(v) for p in polygon for v in p):return False
    roads=[(a+margin,b+margin,c-margin,d-margin) for a,b,c,d in CORRIDORS]
    left=min(p[0] for p in polygon);right=max(p[0] for p in polygon)
    cuts=sorted({left,right,*[v for r in roads for v in (r[0],r[2]) if left<v<right]})
    for a,c in zip(cuts,cuts[1:]):
        if c-a<1e-12:continue
        part=clip_axis(clip_axis(polygon,0,a,True),0,c,False)
        if not part:continue
        low=min(p[1] for p in part);high=max(p[1] for p in part)
        intervals=sorted((b,d) for x,b,z,d in roads if x<(a+c)/2<z)
        merged=[]
        for b,d in intervals:
            if merged and b<=merged[-1][1]+1e-12:merged[-1][1]=max(merged[-1][1],d)
            else:merged.append([b,d])
        if not any(b-1e-12<=low and high<=d+1e-12 for b,d in merged):return False
    return True


def safe_pose(x,y,yaw,margin=.008):
    return polygon_in_road(body_corners(x,y,yaw),margin)

def predict(x,y,yaw,v,w,dt):
    if abs(w)<1e-7:return x+v*dt*math.cos(yaw),y+v*dt*math.sin(yaw),yaw
    return x+v/w*(math.sin(yaw+w*dt)-math.sin(yaw)),y-v/w*(math.cos(yaw+w*dt)-math.cos(yaw)),yaw+w*dt


def clear_of_posts(x, y, yaw, posts, margin=.012):
    """Separating-axis test for chassis rectangle versus known physical post boxes."""
    c,s=math.cos(yaw),math.sin(yaw)
    body_axes=((c,s),(-s,c))
    for post in posts:
        pc,ps=math.cos(post['yaw']),math.sin(post['yaw'])
        post_axes=((pc,ps),(-ps,pc));dx=post['x']-x;dy=post['y']-y
        separated=False
        for ax,ay in body_axes+post_axes:
            body_radius=HALF_LENGTH*abs(ax*c+ay*s)+HALF_WIDTH*abs(-ax*s+ay*c)
            post_radius=post['width']/2*abs(ax*pc+ay*ps)+post['depth']/2*abs(-ax*ps+ay*pc)
            if abs(dx*ax+dy*ay)>body_radius+post_radius+margin:
                separated=True;break
        if not separated:return False
    return True


def corridor_recovery(pose,goal,posts,speed=.06):
    """A short forward move toward the next route goal while holding heading.

    Only use when current body is in-road and every checked forward footprint
    is in-road and clear of posts. No reverse, tolerance relaxation or teleport.
    """
    x,y,yaw=pose
    if not safe_pose(x,y,yaw):return None
    if (goal[0]-x)*math.cos(yaw)+(goal[1]-y)*math.sin(yaw)<=.08:return None
    if all(safe_pose(*predict(x,y,yaw,speed,0.,t)) and
           clear_of_posts(*predict(x,y,yaw,speed,0.,t),posts,margin=.025)
           for t in (.1,.3,.5)):
        return speed,0.
    return None
