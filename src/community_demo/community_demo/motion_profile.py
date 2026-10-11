"""Slow before upcoming bends instead of entering them at straight-line speed."""
import math
import numpy as np


def path_velocity(points,nearest,curvature,remaining,base=.26,tolerance=.02,settings=None):
    """Local speed envelope with curvature and backwards braking constraints.

    All units are metres/seconds. Straight portions of a curved route retain
    normal cruise speed. Future curvature caps propagate backwards using
    v^2 <= v_future^2 + 2*a*distance, rather than a whole-route slow flag.
    The actual steering curvature also caps angular speed and lateral accel.
    """
    if (len(points)<2 or not isinstance(nearest,int) or not 0<=nearest<len(points)-1 or
            not all(math.isfinite(v) for v in (curvature,remaining,base,tolerance)) or
            base<=0 or remaining<0 or tolerance<0 or
            any(len(p)<2 or not all(math.isfinite(v)for v in p[:2])for p in points) or
            not any(math.dist(a[:2],b[:2])>1e-8 for a,b in zip(points,points[1:]))):
        return 0.,{'invalid_geometry':True}
    settings=settings or {};base=min(base,settings.get('cruise',.26))
    angular=settings.get('angular_speed',.65);lateral=settings.get('lateral_acceleration',.20);deceleration=settings.get('braking_acceleration',.30)
    def cap(k):
        k=abs(k)
        return min(base,angular/max(k,.001),math.sqrt(lateral/max(k,.001)))
    local=cap(curvature)
    # The controller owns the Euclidean arrival-circle stop. A projected
    # arc-distance margin must not stop a laterally displaced robot outside it.
    latency=settings.get('command_latency_budget',.25)
    reserve=max(.001,remaining-tolerance*.25)
    braking=math.sqrt((deceleration*latency)**2+2*deceleration*reserve)-deceleration*latency
    preview=base;travel=0.
    for i in range(max(1,nearest),len(points)-1):
        if i>nearest:travel+=math.dist(points[i-1][:2],points[i][:2])
        if travel>.40:break
        a,b,c=points[max(0,i-4)],points[i],points[min(len(points)-1,i+4)]
        ab=math.dist(a[:2],b[:2]);bc=math.dist(b[:2],c[:2]);ac=math.dist(a[:2],c[:2])
        cross=(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        k=2*cross/max(ab*bc*ac,1e-10)
        # The centred geometry window straddles a curvature transition;
        # reserve 3cm so its first full-curvature sample does not brake late.
        preview=min(preview,math.sqrt(cap(k)**2+2*deceleration*max(0.,travel-.03)))
    selected=min(base,local,preview,braking)
    return selected,{'cruise':base,'local_curvature':local,'future_braking':preview,
                     'goal_braking':braking,'remaining_arc_m':remaining,'latency_budget_s':latency,
                     'steering_curvature':curvature,'selected_before_semantic_caps':selected}


def velocity_envelope(poses,settings=None):
    """Retiming shared by straights, all bends, and the final pose connection.

    Curvature derivative limits angular acceleration; backwards squared-speed
    propagation reserves enough distance to reach every upcoming lower limit.
    The final position stop is also checked at runtime with a latency budget.
    """
    settings=settings or {};p=np.asarray(poses,dtype=float)
    if p.ndim!=2 or p.shape[0]<2 or p.shape[1]<3 or not np.isfinite(p).all():return None
    ds=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
    if np.any(ds<1e-9):return None
    distance=np.r_[0,np.cumsum(ds)];yaw=np.unwrap(p[:,2]);k=np.gradient(yaw,distance)
    derivative=np.gradient(k,distance)
    angular=settings.get('angular_speed',.65);aa=settings.get('angular_acceleration',1.2)
    lateral=settings.get('lateral_acceleration',.20);base=settings.get('cruise',.26)
    speeds=np.minimum(base,np.minimum(angular/np.maximum(abs(k),.001),np.sqrt(lateral/np.maximum(abs(k),.001))))
    speeds=np.minimum(speeds,np.sqrt(.75*aa/np.maximum(abs(derivative),.001)))
    # Reserve the rest of angular acceleration for k * dv/dt.
    acceleration=np.minimum(settings.get('braking_acceleration',.30),.2*aa/np.maximum(abs(k),.001))
    for i in range(len(speeds)-2,-1,-1):
        speeds[i]=min(speeds[i],math.sqrt(speeds[i+1]**2+2*min(acceleration[i:i+2])*ds[i]))
    for i in range(1,len(speeds)):
        speeds[i]=min(speeds[i],math.sqrt(speeds[i-1]**2+2*min(acceleration[i-1:i+1])*ds[i-1]))
    return {'arc_m':distance.tolist(),'curvature':k.tolist(),'curvature_derivative':derivative.tolist(),'speed_m_s':speeds.tolist()}


def coupled_command(linear,curvature,previous,dt,settings=None):
    """Slew the command vector together instead of saturating its axes separately."""
    settings=settings or {};angular=linear*curvature
    if not all(math.isfinite(v) for v in (linear,curvature,*previous,dt)) or dt<=0:return 0.,0.
    dt=min(dt,.1);dv=linear-previous[0];dw=angular-previous[1]
    fraction=min(1.,settings.get('linear_acceleration',.5)*dt/max(abs(dv),1e-9),
        settings.get('angular_acceleration',1.2)*dt/max(abs(dw),1e-9))
    return previous[0]+fraction*dv,previous[1]+fraction*dw
