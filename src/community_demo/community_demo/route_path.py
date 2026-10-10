"""Remove redundant Nav2 bends only with whole-body, live-grid clearance.

Input is Nav2's actual path and current costmap. No simulator pose or image is
used. A rejected shortcut retains the original path, never an unchecked chord.
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
