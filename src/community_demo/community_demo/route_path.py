"""Remove redundant Nav2 bends only with whole-body, live-grid clearance.

Input is Nav2's actual path and current costmap. No simulator pose or image is
used. Every replacement is checked; an unavailable safe route is rejected.
"""
import math
from community_demo.road_rules import (
    HALF_LENGTH,HALF_WIDTH,body_corners,clip_axis,polygon_in_road)


def convex_hull(points):
    points=sorted(set(points))
    def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    def half(seq):
        result=[]
        for p in seq:
            while len(result)>1 and cross(result[-2],result[-1],p)<=0:result.pop()
            result.append(p)
        return result
    return half(points)[:-1]+half(points[::-1])[:-1]


def intersects_box(polygon,left,bottom,right,top):
    clipped=polygon
    for axis,bound,greater in ((0,left,True),(0,right,False),(1,bottom,True),(1,top,False)):
        clipped=clip_axis(clipped,axis,bound,greater)
        if not clipped:return False
    # Contact also blocks: the margin has already been added to the body.
    return bool(clipped)


class GridClearance:
    """OccupancyGrid values: 100 lethal, -1 unknown; inflation is not a wall."""
    def __init__(self,width,height,resolution,origin,data):
        self.width=width;self.height=height;self.resolution=resolution
        self.origin=origin;self.data=data

    def clear(self,polygon):
        ox,oy=self.origin;r=self.resolution
        left=math.floor((min(p[0] for p in polygon)-ox)/r)
        right=math.floor((max(p[0] for p in polygon)-ox)/r)
        bottom=math.floor((min(p[1] for p in polygon)-oy)/r)
        top=math.floor((max(p[1] for p in polygon)-oy)/r)
        if left<0 or bottom<0 or right>=self.width or top>=self.height:return False
        for row in range(bottom,top+1):
            for col in range(left,right+1):
                v=self.data[row*self.width+col]
                if (v<0 or v>=100) and intersects_box(polygon,ox+col*r,oy+row*r,ox+(col+1)*r,oy+(row+1)*r):return False
        return True


def clearance_polygon(polygon,grid,posts):
    if not polygon_in_road(polygon):return False
    # Keep the existing 5cm physical post-clearance acceptance target.
    for p in posts:
        # Gantry posts in this field are axis-aligned, including the 90deg pair.
        c,s=abs(math.cos(p['yaw'])),abs(math.sin(p['yaw']))
        rx=(p['width']*c+p['depth']*s)/2+.05
        ry=(p['width']*s+p['depth']*c)/2+.05
        if intersects_box(polygon,p['x']-rx,p['y']-ry,p['x']+rx,p['y']+ry):return False
    return grid.clear(polygon)


def segment_clear(a,b,grid,posts):
    if math.dist(a,b)<1e-8:return False
    yaw=math.atan2(b[1]-a[1],b[0]-a[0])
    swept=convex_hull(body_corners(*a,yaw)+body_corners(*b,yaw))
    return clearance_polygon(swept,grid,posts)


def pivot_clear(point,grid,posts):
    # The square encloses the complete rotation disk, conservative at corners.
    radius=math.hypot(HALF_LENGTH,HALF_WIDTH)
    x,y=point
    return clearance_polygon([(x-radius,y-radius),(x+radius,y-radius),
                              (x+radius,y+radius),(x-radius,y+radius)],grid,posts)


def simplify_path(points,start,grid,posts):
    if not points or grid is None:return None
    points=[start]+[p for p in points[1:] if math.dist(start,p)>.02]
    if len(points)<2 or not pivot_clear(start,grid,posts):return None
    end=points[-1]
    if segment_clear(start,end,grid,posts):return [start,end]
    # A cell-gradient path can hug the outside line so closely that none of its
    # original vertices permits a stationary turn. Rectify one staging point
    # by at most 6cm from a supplied vertex, checking both complete swept legs
    # and the full pivot envelope against the same fresh grid and post margin.
    candidates=[]
    for p in points[1:-1]:
        for stage in ((p[0],start[1]),(start[0],p[1])):
            if math.dist(stage,p)>.06 or min(math.dist(start,stage),math.dist(stage,end))<.12:continue
            if not pivot_clear(stage,grid,posts):continue
            if not segment_clear(start,stage,grid,posts) or not segment_clear(stage,end,grid,posts):continue
            first=math.atan2(stage[1]-start[1],stage[0]-start[0])
            second=math.atan2(end[1]-stage[1],end[0]-stage[0])
            turn=abs(math.atan2(math.sin(second-first),math.cos(second-first)))
            candidates.append((math.dist(start,stage)+math.dist(stage,end)+.1*turn,stage))
    if candidates:return [start,min(candidates)[1],end]
    result=[start];index=0
    while index<len(points)-1:
        for j in range(len(points)-1,index,-1):
            if (j==len(points)-1 or pivot_clear(points[j],grid,posts)) and segment_clear(points[index],points[j],grid,posts):
                result.append(points[j]);index=j;break
        else:return None
    return result


def pose_sweep_clear(a,b,grid,posts):
    """Conservative short arc sweep, including rotation/chord interpolation error."""
    delta=abs(math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2])))
    if math.dist(a[:2],b[:2])>.016 or delta>.10:return False
    # 4mm bounds the sagitta for our <=1.6cm steps and >=8cm turn radii,
    # plus the maximum footprint-corner rotation bulge between samples.
    corners=body_corners(*a)+body_corners(*b)
    swept=convex_hull([(x+dx,y+dy) for x,y in corners
                       for dx,dy in ((-.004,-.004),(-.004,.004),(.004,-.004),(.004,.004))])
    return clearance_polygon(swept,grid,posts)


def continuous_lower_path(points,pose,grid,posts):
    """Tangent quarter-circle, straight gantry pass, then a gentle tangent bend.

    Nav2 still supplies the reachable goal. Candidate turns use measured start
    pose and the live grid, and are never accepted on centre-line checks alone.
    """
    if grid is None or len(points)<2 or not all(math.isfinite(v)for v in pose):return None
    sx,sy,yaw=pose;ex,ey=points[-1]
    if abs(math.atan2(math.sin(yaw+math.pi/2),math.cos(yaw+math.pi/2)))>.15:return None
    if not .8<ex<1.3 or not -1.85<ey<-1.55:return None
    desired=max(.09,min(.15,sy+1.805))
    for radius in (desired,desired-.01,desired+.01,.10,.12,.14):
        if not .08<=radius<=.16:continue
        turn=-yaw;cx=sx-radius*math.sin(yaw);cy=sy+radius*math.cos(yaw)
        steps=max(2,math.ceil(radius*turn/.006));poses=[]
        for i in range(steps+1):
            z=yaw+turn*i/steps;poses.append((cx+radius*math.sin(z),cy-radius*math.cos(z),z))
        ax,ay,_=poses[-1];stage=max(.46,ax+.15)
        if stage>ex-.30:continue
        n=max(1,math.ceil((stage-ax)/.006))
        poses.extend((ax+(stage-ax)*i/n,ay,0.)for i in range(1,n+1))
        # Quadratic starts tangent to the straight and gradually rises to the
        # lane centre. Its heading stays monotonic, avoiding a second S bend.
        control=((stage+ex)/2,ay);n=max(2,math.ceil((ex-stage+abs(ey-ay))/.005))
        for i in range(1,n+1):
            t=i/n;u=1-t;x=u*u*stage+2*u*t*control[0]+t*t*ex
            y=u*u*ay+2*u*t*control[1]+t*t*ey
            dx=2*u*(control[0]-stage)+2*t*(ex-control[0]);dy=2*t*(ey-control[1])
            poses.append((x,y,math.atan2(dy,dx)))
        if all(pose_sweep_clear(a,b,grid,posts)for a,b in zip(poses,poses[1:])):return poses
    return None


def curve_target(points,pose,progress):
    """Monotonic nearest segment and arc-length carrot for pure pursuit."""
    x,y,yaw=pose
    nearest=min(range(max(0,progress-2),len(points)-1),
                key=lambda i:math.dist((x,y),points[i][:2]))
    progress=max(progress,nearest);a,b=points[progress],points[progress+1]
    dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
    along=max(0.,min(length,((x-a[0])*dx+(y-a[1])*dy)/length))
    # Tight turn: a short carrot; following straight: longer, smoother tracking.
    heading=math.atan2(dy,dx);look=.075 if abs(heading)>.35 else .14
    remaining=look+along;i=progress
    while i<len(points)-2 and remaining>math.dist(points[i][:2],points[i+1][:2]):
        remaining-=math.dist(points[i][:2],points[i+1][:2]);i+=1
    a,b=points[i],points[i+1];length=math.dist(a[:2],b[:2]);f=min(1.,remaining/max(length,1e-9))
    gx,gy=a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1])
    error=math.atan2(math.sin(math.atan2(gy-y,gx-x)-yaw),math.cos(math.atan2(gy-y,gx-x)-yaw))
    curvature=2*math.sin(error)/max(math.hypot(gx-x,gy-y),.025)
    return error,curvature,progress
