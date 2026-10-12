"""Join transit waypoints into a certified path ending at a semantic stop."""
import math
import copy
from community_demo.trajectory import plan_trajectory,tangent_connector,certify


def batch_end(waypoints,index):
    while index<len(waypoints)-1 and not waypoints[index]['views'] and not waypoints[index]['stop_for_light']:
        index+=1
    return index


def passage(previous,current,waypoint):
    """Measured crossing of a transit plane, with the existing 2cm lane limit.

    A 10Hz controller can cross an 8mm arrival circle between samples. Transit
    points therefore use the measured segment, not a requirement to stop inside
    that circle. Reject sideways crossings, reverse motion and pose jumps.
    """
    x,y=waypoint['position'];h=waypoint['heading'];c,s=math.cos(h),math.sin(h)
    angle=math.atan2(math.sin(current[2]-h),math.cos(current[2]-h))
    if math.cos(angle)<=0 or math.dist(previous[:2],current[:2])>.08:return False
    a=(previous[0]-x)*c+(previous[1]-y)*s
    b=(current[0]-x)*c+(current[1]-y)*s
    if math.dist(current[:2],(x,y))<waypoint['tolerance']:return True
    if not a<=0<=b or b-a<1e-8:return False
    f=-a/(b-a)
    px=previous[0]+f*(current[0]-previous[0]);py=previous[1]+f*(current[1]-previous[1])
    return abs(-(px-x)*s+(py-y)*c)<=.02


def plan_batch(reference,start,waypoints,begin,end,grid,posts,settings):
    """Each future join is explicit; only the first pose is a measured pose."""
    curve=[];segments=[];entry=start;cursor=0
    for index in range(begin,end+1):
        w=waypoints[index];goal=w['position'];heading=w['heading']
        if math.dist(entry[:2],goal)<1e-5:
            if abs(math.atan2(math.sin(entry[2]-heading),math.cos(entry[2]-heading)))>1e-4:
                return None,{'rejected':'coincident waypoint changes heading'}
            segments.append({'waypoint_index':index,'curve_end_index':len(curve)-1,'coincident':True})
            continue
        nearest=min(range(cursor,len(reference)),key=lambda j:math.dist(reference[j][:2],goal))
        part=list(reference[cursor:nearest+1])+[tuple(goal)]
        if len(part)<2:part=[entry[:2],tuple(goal)]
        segment,meta=plan_trajectory(part,entry,heading,grid,posts,settings)
        if segment is None:return None,{'rejected':'uncertified batch segment','waypoint_index':index,'detail':meta}
        curve.extend(segment[1:] if curve else segment)
        segments.append({'waypoint_index':index,'curve_end_index':len(curve)-1,'planning':meta})
        entry=curve[-1];cursor=nearest
    if len(curve)<2:return None,{'rejected':'empty batch'}
    first=next((s['planning']for s in segments if 'planning'in s),{})
    return curve,{'family':'continuous_semantic_batch','begin_index':begin,'end_index':end,
                  'segments':segments,'initial_centre_join_end_index':first.get('initial_centre_join_end_index',0),
                  'goal':curve[-1],'source':'Measured start followed by explicit planned tangent joins; every segment body sweep certified'}

def rebase_batch(curve,metadata,start,grid,posts,minimum_radius=.08):
    if not curve or math.dist(start[:2],curve[0][:2])>.04 or abs(math.atan2(math.sin(start[2]-curve[0][2]),math.cos(start[2]-curve[0][2])))>.06:
        return None,{'rejected':'cached start outside checked handover envelope'}
    limit=next(s['curve_end_index']for s in metadata['segments']if not s.get('coincident'))
    # Keep the first mandatory via point; use a short tangent connection on
    # its preceding straight. All later path positions/headings remain exact.
    candidates=[]
    for i,p in enumerate(curve[:limit+1]):
        length=math.dist(start[:2],p[:2])
        if .06<=length<=.20 and abs(math.atan2(math.sin(p[2]-curve[0][2]),math.cos(p[2]-curve[0][2])))<.03:
            candidates.append(i)
    for i in reversed(candidates[::max(1,len(candidates)//5)]):
        join=tangent_connector(start,curve[i][:2],curve[i][2])
        k=[abs(math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2])))/max(1e-9,math.dist(a[:2],b[:2]))for a,b in zip(join,join[1:])]
        if max(k,default=math.inf)>1/minimum_radius:continue
        combined=join+curve[i+1:]
        if not certify(combined,grid,posts):continue
        meta=copy.deepcopy(metadata);shift=len(join)-1-i
        for segment in meta['segments']:segment['curve_end_index']+=shift
        meta['initial_centre_join_end_index']=max(len(join)-1,meta.get('initial_centre_join_end_index',0)+shift)
        meta['handover']={'cached_start':curve[0],'measured_start':start,'anchor_index':i,'checked_join_points':len(join),'fresh_grid_recertified':True,'original_family':meta['family']}
        return combined,meta
    return None,{'rejected':'no certified tangent handover preserving first via point'}
