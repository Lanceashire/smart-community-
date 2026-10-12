"""One forward, G2 trajectory solver for every task waypoint.

Nav2 proves reachability. Candidate straight / clothoid / circular primitives
are solved against the same measured start and exact goal pose, then checked
against the full footprint, live grid, unknown cells and configured roads.
No route name or field coordinate participates in the geometry algorithm.
"""
import math
import numpy as np
from scipy.optimize import least_squares
from community_demo.route_path import pose_sweep_clear,terminal_sweep_clear,clearance_polygon
from community_demo.road_rules import body_corners,CORRIDORS

def wrap(z):return math.atan2(math.sin(z),math.cos(z))

# Five-point Gauss integration avoids introducing sample-dependent endpoint drift.
GX,GW=np.polynomial.legendre.leggauss(5)
GQUAD=tuple(zip(GX.tolist(),GW.tolist()))
def ramp(p,k0,k1,length,dense=False):
    if length<1e-8:return p,[]
    if not dense:
        # The nonlinear fit needs only the endpoint. Keep identical quadrature
        # without allocating NumPy arrays for each finite-difference residual.
        scale=length/2;dx=dy=0.
        for node,weight in GQUAD:
            ss=(1+node)*scale
            theta=p[2]+k0*ss+(k1-k0)*ss*ss/(2*length)
            dx+=weight*math.cos(theta);dy+=weight*math.sin(theta)
        end=(p[0]+scale*dx,p[1]+scale*dy,p[2]+(k0+k1)*length/2)
        return end,[end]
    n=max(1,math.ceil(length/.005))
    ts=np.arange(n+1)*length/n; aa=ts[:-1,None];bb=ts[1:,None]
    ss=(aa+bb)/2+(bb-aa)/2*GX
    theta=p[2]+k0*ss+(k1-k0)*ss*ss/(2*length)
    dx=np.sum(np.cos(theta)*GW,axis=1)*length/(2*n)
    dy=np.sum(np.sin(theta)*GW,axis=1)*length/(2*n)
    headings=p[2]+k0*ts[1:]+(k1-k0)*ts[1:]**2/(2*length)
    poses=list(zip(p[0]+np.cumsum(dx),p[1]+np.cumsum(dy),headings))
    return poses[-1],poses

def constant(p,k,length,dense=False):
    if length<1e-8:return p,[]
    if not dense:
        theta=p[2]+k*length
        if abs(k)<1e-8:end=(p[0]+length*math.cos(p[2]),p[1]+length*math.sin(p[2]),theta)
        else:end=(p[0]+(math.sin(theta)-math.sin(p[2]))/k,p[1]-(math.cos(theta)-math.cos(p[2]))/k,theta)
        return end,[end]
    n=max(1,math.ceil(length/.005))
    ss=np.arange(1,n+1)*length/n;theta=p[2]+k*ss
    if abs(k)<1e-8:xx=p[0]+ss*math.cos(p[2]);yy=p[1]+ss*math.sin(p[2])
    else:xx=p[0]+(np.sin(theta)-math.sin(p[2]))/k;yy=p[1]-(np.cos(theta)-math.cos(p[2]))/k
    poses=list(zip(xx,yy,theta));return poses[-1],poses

def arc(p,sign,angle,r,dense=False):
    transition=min(.10,r*.35);maximum=1/r
    if angle<maximum*transition:
        transition=math.sqrt(max(0.,angle)*transition/maximum)
        peak=sign*maximum*transition/min(.10,r*.35);middle=0.
    else:peak=sign*maximum;middle=angle*r-transition
    result=[]
    for k0,k1,ll in [(0,peak,transition),(peak,peak,middle),(peak,0,transition)]:
        p,poses=(constant(p,k0,ll,dense) if k0==k1 else ramp(p,k0,k1,ll,dense));result.extend(poses)
    return p,result

def integrate_primitives(start,signs,r,lengths,dense=False):
    p=list(start);out=[tuple(p)]
    p,z=arc(p,signs[0],lengths[0],r[0],dense);out.extend(z)
    p,z=constant(p,0,lengths[1],dense);out.extend(z)
    p,z=arc(p,signs[1],lengths[2],r[1],dense);out.extend(z)
    return p,out

def seed(start,goal,signs,r):
    x,y,h=start;ex,ey,g=goal;s0,s1=signs
    c0=np.array((x-s0*r[0]*math.sin(h),y+s0*r[0]*math.cos(h)))
    c1=np.array((ex-s1*r[1]*math.sin(g),ey+s1*r[1]*math.cos(g)))
    delta=c1-c0;d=float(np.linalg.norm(delta));offset=s1*r[1]-s0*r[0]
    if d<abs(offset)+1e-7:return None
    tangent=math.atan2(delta[1],delta[0])-math.asin(offset/d)
    a=(s0*(tangent-h))%(2*math.pi);b=(s1*(g-tangent))%(2*math.pi)
    if max(a,b)>math.pi+.15:return None
    return [max(1e-7,a),math.sqrt(d*d-offset*offset),max(1e-7,b)]

def solve(start,goal,signs,r):
    guess=seed(start,goal,signs,r)
    if guess is None:return None
    target_heading=start[2]+signs[0]*guess[0]+signs[1]*guess[2]
    def residual(v):
        p,_=integrate_primitives(start,signs,r,v)
        return [p[0]-goal[0],p[1]-goal[1],p[2]-target_heading]
    fit=least_squares(residual,guess,bounds=([0,0,0],[math.pi+.15,10,math.pi+.15]),max_nfev=45,xtol=1e-10,ftol=1e-10,gtol=1e-10)
    if np.linalg.norm(residual(fit.x))>1e-5:return None
    return integrate_primitives(start,signs,r,fit.x,True)[1]


def tangent_connector(start,end,heading):
    """Quintic position/tangent connection with zero endpoint curvature."""
    length=math.dist(start[:2],end);handle=length/5
    u=(math.cos(start[2]),math.sin(start[2]));v=(math.cos(heading),math.sin(heading))
    controls=[start[:2],(start[0]+handle*u[0],start[1]+handle*u[1]),
        (start[0]+2*handle*u[0],start[1]+2*handle*u[1]),
        (end[0]-2*handle*v[0],end[1]-2*handle*v[1]),
        (end[0]-handle*v[0],end[1]-handle*v[1]),end]
    n=max(2,math.ceil(length/.004));result=[]
    for i in range(n+1):
        t=i/n;z=1-t
        weights=[math.comb(5,j)*z**(5-j)*t**j for j in range(6)]
        x=sum(w*p[0] for w,p in zip(weights,controls));y=sum(w*p[1] for w,p in zip(weights,controls))
        dw=[5*math.comb(4,j)*z**(4-j)*t**j for j in range(5)]
        dx=sum(w*(b[0]-a[0]) for w,a,b in zip(dw,controls,controls[1:]))
        dy=sum(w*(b[1]-a[1]) for w,a,b in zip(dw,controls,controls[1:]))
        result.append((x,y,math.atan2(dy,dx)))
    return result


def certify(curve,grid,posts):
    return (len(curve)>1 and
        all(clearance_polygon(body_corners(*z),grid,posts) for z in curve[::4]) and
        terminal_sweep_clear(curve[-1],grid,posts) and
        all(pose_sweep_clear(a,b,grid,posts) for a,b in zip(curve,curve[1:])))


def plan_trajectory(points,start,goal_heading,grid,posts,settings=None):
    """Return certified poses and diagnostics, or reject without fallback motion.

    A checked initial tangent within the Nav2 corridor is retained before
    solving a changed goal heading. Nav2 smoothing may tilt toward a lateral
    goal early; that must not bend a whole preceding straight. All paths use
    the identical primitive
    families and solve. Endpoint positions and headings are never moved.
    """
    if (grid is None or len(points)<2 or len(start)!=3 or
        not all(math.isfinite(v) for v in (*start,goal_heading)) or
        any(len(p)<2 or not all(math.isfinite(v) for v in p[:2]) for p in points)):
        return None,{'rejected':'missing or invalid geometry'}
    settings=settings or {};minimum=settings.get('minimum_radius',.08)
    maximum=settings.get('maximum_radius',.44);step=settings.get('radius_step',.01)
    window=settings.get('terminal_transition_window',.75)
    prefix_deviation=settings.get('reference_prefix_deviation',.15)
    if not all(math.isfinite(z) and z>0 for z in (minimum,maximum,step,window,prefix_deviation)) or minimum>maximum or step<.005:
        return None,{'rejected':'invalid solver bounds'}
    goal=(*points[-1][:2],goal_heading);distance=math.dist(start[:2],goal[:2])
    if distance<1e-5:return None,{'rejected':'no translational path'}
    bearing=math.atan2(goal[1]-start[1],goal[0]-start[0])
    if abs(wrap(bearing-start[2]))<.06 and abs(wrap(bearing-goal_heading))<.06:
        line=tangent_connector(start,goal[:2],goal_heading)
        if certify(line,grid,posts):return line,{'family':'straight','candidate_count':1,'goal':goal}
    # Stay within the reachable reference corridor, then certify every swept
    # footprint against the live grid. An offset Nav2 centreline is not a
    # reason to gradually leave an otherwise clear initial lane tangent.
    # A turning goal is beside the lane. Its direct bearing must not tilt the
    # preceding straight: follow the reachable Nav2 prefix in the start frame.
    # Use the configured corridor's centre/tangent for a long straight, rather
    # than accumulating the small measured entry-heading error over metres.
    # These are supplied road polygons, not a waypoint or field-coordinate
    # template. The initial pose still connects continuously and every sweep
    # is certified against the live grid before this hint may be used.
    axis_yaw=start[2];centre=start[:2];hints=[]
    for left,bottom,right,top in (CORRIDORS if grid.roads is None else grid.roads):
        if not left<=start[0]<=right or not bottom<=start[1]<=top:continue
        width,height=right-left,top-bottom
        if max(width,height)<2*min(width,height):continue
        heading=0. if width>height else math.pi/2
        if math.cos(heading-start[2])<0:heading+=math.pi
        offset=abs(start[1]-(bottom+top)/2) if width>height else abs(start[0]-(left+right)/2)
        if abs(wrap(heading-start[2]))>.06 or offset>.04:continue
        point=(start[0],(bottom+top)/2) if width>height else ((left+right)/2,start[1])
        hints.append((offset,heading,point))
    if hints:_,axis_yaw,centre=min(hints,key=lambda v:v[0])
    axis=(math.cos(axis_yaw),math.sin(axis_yaw));available=0.
    for p in points:
        dx,dy=p[0]-start[0],p[1]-start[1];along=dx*axis[0]+dy*axis[1]
        if abs(dx*axis[1]-dy*axis[0])>prefix_deviation:break
        available=max(available,along)
    goal_along=(goal[0]-start[0])*axis[0]+(goal[1]-start[1])*axis[1]
    aligned=available>window and goal_along>window
    advance=max(0.,min(available,goal_along-window)) if aligned else 0.
    prefix=[];entry=tuple(start);join_end_index=0
    if advance>.02:
        correction=min(.25,advance)
        point=(centre[0]+axis[0]*correction,centre[1]+axis[1]*correction)
        prefix=tangent_connector(start,point,axis_yaw)
        join_end_index=len(prefix)-1
        end=(centre[0]+axis[0]*advance,centre[1]+axis[1]*advance)
        _,line=constant(prefix[-1],0,advance-correction,True);prefix.extend(line)
        entry=(*end,axis_yaw)
    candidates=[];tried=0;visited=set()
    radii_values=list(np.arange(minimum,maximum+step*.25,step))
    ratios=(.65,.8,1.,1.25,1.55)
    def search(indices,selected_ratios):
        nonlocal tried
        for index in indices:
            radius=radii_values[index]
            for ratio in selected_ratios:
                for signs in ((1,1),(-1,-1),(1,-1),(-1,1)):
                    key=(index,ratio,signs)
                    if key in visited:continue
                    radii=(float(radius),float(radius)*ratio)
                    if not minimum<=radii[1]<=maximum+1e-8:continue
                    visited.add(key)
                    curve=solve(entry,goal,signs,radii);tried+=1
                    if curve is None:continue
                    curve=prefix+curve[1:] if prefix else curve
                    if not certify(curve,grid,posts):continue
                    duration=0.;deviation=0.;length=0.;max_k=0.
                    for a,b in zip(curve,curve[1:]):
                        ds=math.dist(a[:2],b[:2]);k=abs(wrap(b[2]-a[2]))/max(ds,1e-10)
                        duration+=ds/min(.26,.65/max(k,.001));length+=ds;max_k=max(max_k,k)
                        # Geometric distance to the reachable Nav2 polyline, invariant
                        # under rotation, translation and reflection of the scene.
                        deviation+=ds*min(math.dist(a[:2],q[:2]) for q in points[::max(1,len(points)//30)])**2
                    meta={'family':'clothoid-circle-straight','radii':radii,'signs':signs,
                          'preserved_straight_prefix_m':advance,'initial_centre_join_end_index':join_end_index,
                          'estimated_duration_s':duration,
                          'length_m':length,'maximum_curvature':max_k,'goal':goal}
                    candidates.append((duration+12*deviation,curve,meta))
    # Search broad radii first, then refine the best safe family locally.
    # All returned candidates retain the identical dense live-grid/whole-body
    # certificate. If the coarse set finds no route, exhaust the original set.
    # Avoid solving hundreds of nearly identical routes while the robot waits
    # at a bend; there are no waypoint names or scene-coordinate branches.
    coarse=sorted(set(range(0,len(radii_values),2))|{len(radii_values)-1})
    for ratio in (1.,.8,1.25):
        search(coarse,(ratio,))
        if candidates:break
    if candidates:
        best=min(candidates,key=lambda x:x[0])[2]
        index=min(range(len(radii_values)),key=lambda i:abs(radii_values[i]-best['radii'][0]))
        ratio=best['radii'][1]/best['radii'][0]
        ratio=min(ratios,key=lambda r:abs(r-ratio))
        search(range(max(0,index-2),min(len(radii_values),index+3)),(ratio,))
    else:
        search(range(len(radii_values)),ratios)
    if not candidates:return None,{'rejected':'no certified primitive path','candidate_count':tried}
    _,curve,meta=min(candidates,key=lambda x:x[0]);meta['candidate_count']=tried
    return curve,meta
