"""Independent moving-junction integration and observation source counterexamples."""
import argparse,json,math
from pathlib import Path
from community_demo.continuous_route import batch_end,passage,plan_batch
from community_demo.task_geometry import TASK
from community_demo.route_path import GridClearance,curve_target,pose_sweep_clear
from community_demo.motion_profile import path_velocity,velocity_envelope,coupled_command
from community_demo.inspection_policy import observation_frame_valid

p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args()
root=Path(__file__).resolve().parents[1];checks={};details=[];wps=TASK['waypoints']
posts=json.loads((root/'src/community_demo/maps/planning-obstacles.json').read_text())['posts']
grid=GridClearance(210,210,.02,(-2.1,-2.1),[0]*44100)
for begin,start,corner in [(3,(-.73,1.7,math.pi),(-1.7,1.7)),(5,(-1.7,1.05,-math.pi/2),(-1.7,.5)),
                           (7,(-.91,.5,0),(-.1,.5)),(11,(-.1,-.26,-math.pi/2),(-.1,-1.7))]:
    end=batch_end(wps,begin);reference=[start[:2],wps[begin]['position'],corner,wps[end]['position']]
    curve,meta=plan_batch(reference,start,wps,begin,end,grid,posts,TASK['trajectory'])
    label=wps[begin]['name'];checks[label+'_path_certified']=curve is not None
    if curve is None:details.append({'case':label,'failure':meta});continue
    checks[label+'_exact_initial_pose']=math.dist(curve[0][:2],start[:2])<1e-9
    checks[label+'_entire_join_swept']=all(pose_sweep_clear(a,b,grid,posts)for a,b in zip(curve,curve[1:]))
    env=velocity_envelope(curve);points=[z[:2]for z in curve];pose=start;command=(0.,0.);progress=0;crossed=False;min_speed=1.;safe=True
    for tick in range(2500):
        old=pose
        _,k,progress=curve_target(points,pose,progress,curve[-1][2],wps[end]['tolerance'])
        a,b=points[progress:progress+2];ds=math.dist(a,b)
        along=max(0.,min(ds,((pose[0]-a[0])*(b[0]-a[0])+(pose[1]-a[1])*(b[1]-a[1]))/ds))
        remaining=ds-along+sum(math.dist(a,b)for a,b in zip(points[progress+1:],points[progress+2:]))
        v,_=path_velocity(points,progress,k,remaining,tolerance=wps[end]['tolerance'])
        v,w=coupled_command(min(v,env['speed_m_s'][progress]),k,command,.1);command=v,w
        h=pose[2]+w*.1
        pose=(pose[0]+(v/w*(math.sin(h)-math.sin(pose[2]))if abs(w)>1e-9 else v*.1*math.cos(h)),
              pose[1]+(-v/w*(math.cos(h)-math.cos(pose[2]))if abs(w)>1e-9 else v*.1*math.sin(h)),h)
        if math.dist(pose[:2],wps[begin]['position'])<.06:
            min_speed=min(min_speed,v)
            samples=[tuple(a+(b-a)*j/4 for a,b in zip(old,pose))for j in range(5)]
            safe=safe and all(pose_sweep_clear(a,b,grid,posts)for a,b in zip(samples,samples[1:]))
        crossed=crossed or passage(old,pose,wps[begin])
        if math.dist(pose[:2],curve[-1][:2])<.02:break
    checks[label+'_measured_passage_detected']=crossed
    checks[label+'_no_stop_at_transit_join']=min_speed>.02
    checks[label+'_actual_join_sweep_safe']=safe
    details.append({'case':label,'minimum_join_speed_m_s':min_speed,'ticks':tick,'passage':crossed})
w={'position':[0,0],'heading':0.,'tolerance':.008}
checks['cross_between_samples_without_stopping']=passage((-.015,.005,0),(.015,.005,0),w)
checks['lateral_miss_rejected']=not passage((-.015,.03,0),(.015,.03,0),w)
checks['pose_jump_rejected']=not passage((-.2,0,0),(.2,0,0),w)
checks['reverse_crossing_rejected']=not passage((.015,0,0),(-.015,0,0),w)
checks['wrong_heading_rejected']=not passage((-.015,0,math.pi),(.015,0,math.pi),w)
valid=lambda source,opened,enc,now=10_050_000_000:observation_frame_valid(source,opened,enc,1.5,now)
checks['moving_capture_with_measured_angle']=valid(10_000_000_000,9_900_000_000,(10.,1.5,0.))
checks['previous_view_frame_rejected']=not valid(9_800_000_000,9_900_000_000,(9.8,1.5,0.))
checks['sweeping_camera_rejected']=not valid(10_000_000_000,9_900_000_000,(10.,1.5,.4))
checks['wrong_camera_angle_rejected']=not valid(10_000_000_000,9_900_000_000,(10.,0.,0.))
checks['stale_encoder_rejected']=not valid(10_000_000_000,9_900_000_000,(9.8,1.5,0.))
checks['stale_frame_rejected']=not valid(10_000_000_000,9_900_000_000,(10.,1.5,0.),10_600_000_000)
checks['missing_encoder_rejected']=not valid(10_000_000_000,9_900_000_000,None)
result={'passed':all(checks.values()),'checks':checks,'details':details,'scope':'Command integration and adversarial source tests; separate real Gazebo and recognition completeness required.'}
out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed']else 1)
