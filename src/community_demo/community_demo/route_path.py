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
    def __init__(self,width,height,resolution,origin,data,roads=None):
        self.width=width;self.height=height;self.resolution=resolution
        self.origin=origin;self.data=data;self.roads=roads

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
    if not polygon_in_road(polygon,corridors=grid.roads):return False
    # Plan with 6.5cm to allow tracking error; retain the 5cm actual-motion gate.
    for p in posts:
        # Gantry posts in this field are axis-aligned, including the 90deg pair.
        c,s=abs(math.cos(p['yaw'])),abs(math.sin(p['yaw']))
        rx=(p['width']*c+p['depth']*s)/2+.065
        ry=(p['width']*s+p['depth']*c)/2+.065
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


def terminal_sweep_clear(pose,grid,posts):
    """Certify the 14cm steering-only tangent continuation as well."""
    x,y,yaw=pose;n=24
    extension=[(x+.14*i/n*math.cos(yaw),y+.14*i/n*math.sin(yaw),yaw)for i in range(n+1)]
    return all(pose_sweep_clear(a,b,grid,posts)for a,b in zip(extension,extension[1:]))


def curve_target(points,pose,progress,terminal_yaw=None,terminal_tolerance=.02):
    """Monotonic nearest segment and arc-length carrot for pure pursuit."""
    x,y,yaw=pose
    nearest=min(range(max(0,progress-2),len(points)-1),
                key=lambda i:math.dist((x,y),points[i][:2]))
    progress=max(progress,nearest);a,b=points[progress],points[progress+1]
    dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
    along=max(0.,min(length,((x-a[0])*dx+(y-a[1])*dy)/length))
    # Tight turn: a short carrot; following straight: longer, smoother tracking.
    # Curvature is geometric, not an absolute field heading. Rotate a route
    # and tracking should be unchanged; avoid the old 7.5/14cm hard switch.
    j=min(len(points)-2,progress+8);before=max(0,progress-4)
    u,v=points[before],points[before+1];q,r=points[j],points[j+1]
    h1=math.atan2(v[1]-u[1],v[0]-u[0]);h2=math.atan2(r[1]-q[1],r[0]-q[0])
    change=abs(math.atan2(math.sin(h2-h1),math.cos(h2-h1)))
    span=sum(math.dist(points[k][:2],points[k+1][:2])for k in range(before,j+1))
    local_curvature=change/max(span,.006)
    # A broad bend needs enough preview for localization and angular-slew
    # latency. Preserve the 7.5cm carrot required by the narrow gantry bends.
    look=.18-.105*min(1.,local_curvature/8.)
    to_end=length-along+sum(math.dist(a[:2],b[:2])for a,b in zip(points[progress+1:],points[progress+2:]))
    if terminal_yaw is not None:
        # Increase terminal heading feedback gradually. A long straight carrot
        # near the endpoint otherwise leaves a turn for stationary alignment.
        # Tight bends need early heading convergence; short straight arrivals
        # retain their longer, less sensitive carrot. Measure the last 30cm of
        # path geometry, independent of route names and field orientation.
        terminal_span=terminal_turn=0.;heading=terminal_yaw
        for u,v in reversed(list(zip(points,points[1:]))):
            span=math.dist(u[:2],v[:2])
            if span<1e-9:continue
            tangent=math.atan2(v[1]-u[1],v[0]-u[0])
            terminal_turn+=abs(math.atan2(math.sin(tangent-heading),math.cos(tangent-heading)))
            terminal_span+=span;heading=tangent
            if terminal_span>=.30:break
        turn_weight=max(0.,min(1.,(terminal_turn-.70)/.35))
        terminal_look=.075-.035*turn_weight
        blend=max(0.,min(1.,(.30-to_end)/.20));blend=blend*blend*(3-2*blend)
        look=look*(1-blend)+min(look,terminal_look)*blend
    remaining=look+along;i=progress
    while i<len(points)-2 and remaining>math.dist(points[i][:2],points[i+1][:2]):
        remaining-=math.dist(points[i][:2],points[i+1][:2]);i+=1
    a,b=points[i],points[i+1];length=math.dist(a[:2],b[:2]);f=min(1.,remaining/max(length,1e-9))
    gx,gy=a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1])
    if terminal_yaw is not None and i==len(points)-2 and remaining>length:
        # Steering-only tangent continuation avoids aiming backwards at a
        # nearly reached endpoint. The robot still stops at the real goal.
        extra=min(.14,remaining-length)
        gx=points[-1][0]+extra*math.cos(terminal_yaw);gy=points[-1][1]+extra*math.sin(terminal_yaw)
    error=math.atan2(math.sin(math.atan2(gy-y,gx-x)-yaw),math.cos(math.atan2(gy-y,gx-x)-yaw))
    curvature=2*math.sin(error)/max(math.hypot(gx-x,gy-y),.025)
    if terminal_yaw is not None and terminal_tolerance>0:
        # Once lateral error fits the existing arrival circle, finish the
        # configured heading while braking. A late localization correction
        # must not pull the carrot across that heading and require a reverse
        # stationary pivot. Retain lateral pursuit outside the allowed room;
        # all original arrival, rolling-time and body-sweep guards still apply.
        lateral=-(x-points[-1][0])*math.sin(terminal_yaw)+(y-points[-1][1])*math.cos(terminal_yaw)
        room=max(0.,min(1.,(terminal_tolerance-abs(lateral))/(terminal_tolerance*.5)))
        blend=max(0.,min(1.,(.10-to_end)/.07));blend=blend*blend*(3-2*blend)*room*(1-turn_weight)
        heading_error=math.atan2(math.sin(terminal_yaw-yaw),math.cos(terminal_yaw-yaw))
        heading_curvature=2*math.sin(heading_error)/max(look,.075)
        curvature=curvature*(1-blend)+heading_curvature*blend
    return error,curvature,progress


def rolling_terminal_allowed(pose,goal,goal_yaw,tolerance,grid,posts,elapsed,*,speed=0.,curvature=0.):
    """Bounded moving heading completion inside the unchanged arrival circle."""
    if grid is None or not all(math.isfinite(v)for v in (*pose,*goal,goal_yaw,tolerance,elapsed,speed,curvature)):
        return False
    x,y,yaw=pose
    error=abs(math.atan2(math.sin(goal_yaw-yaw),math.cos(goal_yaw-yaw)))
    passed=(x-goal[0])*math.cos(goal_yaw)+(y-goal[1])*math.sin(goal_yaw)
    if not (0<=elapsed<1.2 and math.dist(pose[:2],goal)<tolerance and
            .025<error<.25 and passed<.015):return False
    # At a narrow arrival circle the braking envelope can leave only a few
    # mm/s. Pure pursuit then needs several seconds to remove a small heading
    # error and creeps out of the circle. Certify that this actual command can
    # finish the heading inside both the time and distance budgets; otherwise
    # the caller stops and checks a stationary pivot at the reached position.
    heading_error=math.atan2(math.sin(goal_yaw-yaw),math.cos(goal_yaw-yaw))
    rate=math.copysign(1.,heading_error)*speed*curvature
    if speed<=0 or rate<=0:return False
    align_time=1.2*error/rate*math.log(error/.025)
    lateral=-(x-goal[0])*math.sin(goal_yaw)+(y-goal[1])*math.cos(goal_yaw)
    forward_room=math.sqrt(max(0.,tolerance*tolerance-lateral*lateral))-passed
    if elapsed+align_time>=1.2 or speed*(align_time+.25)>=forward_room:return False
    return (clearance_polygon(body_corners(*pose),grid,posts) and
            terminal_sweep_clear((*goal,goal_yaw),grid,posts))
