#!/usr/bin/env python3
"""Summarize final-code physical laps; retain preliminary and previous failed evidence."""
import hashlib,json,math
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.font_manager import FontProperties

root=Path(__file__).resolve().parents[1];folder=root/'validation/patrol-optimization-2026-10-02'
reports=[]
for scenario,expected in [('main',4),('start-plus',1),('start-minus',1)]:
    files=sorted((folder/scenario).glob('lap-[0-9][0-9][0-9].json'))
    if len(files)!=expected:raise RuntimeError(f'{scenario}: expected {expected} completed laps, found {len(files)}')
    for file in files:
        data=json.loads(file.read_text());data['scenario']=scenario;data['report']=str(file.relative_to(folder));reports.append(data)
regression=json.loads((folder/'controller-regression.json').read_text())
retry=json.loads((folder/'retry-probe.json').read_text())
inputs=json.loads((folder/'unchanged-inputs.json').read_text());hashes=json.loads((folder/'final-code-hashes.json').read_text())
assert all(hashlib.sha256((root/p).read_bytes()).hexdigest()==h for p,h in {**inputs,**hashes}.items()),'Audited runtime source/input changed'
compact=[]
for d in reports:
    green_wait=sum(r['status'].startswith(('STOP top_stop','STOP lower_stop')) and r['phase']=='green' for r in d['records'])
    compact.append({k:d[k] for k in ['scenario','report','run_id','lap_id','mode','passed','checks','physical_distance_m','sim_elapsed_s','counts','plates','max_fresh_position_error_m','max_fresh_yaw_error_deg','road_violation_samples','red_waits','whole_body_crossings']} | {
        'waypoint_count':len(d['waypoints']),'observation_count':len(d['observations']),
        'events':len(d['perception']['events']),'confirmed_views':sum(x['confirmed'] for x in d['observation_outcomes']),
        'retry_views':[{'view':x['view'],'attempt':x['attempt'],'elapsed_s':x['elapsed_s'],'confirmed':x['confirmed']} for x in d['observation_outcomes'] if x['attempt']],
        'green_wait_at_stop_samples':green_wait,
        'faults':[{'node':f['node'],'passed':f['passed'],'pause_s':f['duration_wall_s'],'settled_speed_max':f['settled_speed_max']} for f in d['faults']],
        'raw_topic_events_seen':d['raw_image_topic_event_frames_seen']})
summary={'review_time':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
    'passed':all(d['passed'] for d in reports) and regression['passed'] and retry['passed'],
    'completed_laps':len(reports),'normal_laps':sum(d['mode']=='normal autonomous lap' for d in reports),
    'controlled_fault_laps':sum(d['mode']=='fault injection' for d in reports),
    'all_edges_green':all(d['checks']['front_and_rear_green'] for d in reports),
    'road_violation_samples':sum(d['road_violation_samples'] for d in reports),
    'checked_event_count':sum(len(d['perception']['events']) for d in reports),
    'confirmed_observations':sum(sum(x['confirmed'] for x in d['observation_outcomes']) for d in reports),
    'retry_probe':retry,'controller_regression':regression,'laps':compact,
    'model':'YOLO11s comparison candidate; unchanged weights and thresholds',
    'source_hashes':hashes,'unchanged_inputs':inputs,
    'previous_strict_failure':'baseline-summary.json: normal lower gate rear cleared on yellow; retained unchanged',
    'preliminary':'preliminary/: initial recovery lap retained; retry-at-completion bookkeeping then corrected; only final-code laps counted above',
    'scope':['Supplied identities, three supplied plates, unchanged fixed layout',
        'Three normal restart timings: late green, red and yellow',
        'Two initial spawn offsets: x/y +/- 0.025 m, yaw +/- 0.035 rad; nominal localization prior unchanged',
        'Three 2.5-second node pauses plus a 7-second perception pause at a stationary plate view',
        'No real camera, unseen identity, arbitrary starting pose, GPU-reset or dynamic-obstacle coverage',
        'Read-only simulator truth for audit; runtime does not subscribe to truth or simulator signal state',
        'View confirmation requires repeated visible evidence; cannot prove absence of unseen/occluded targets',
        'Road and stop-line checks use sampled true body footprint; best-effort raw image drops remain diagnostic'],
    'photos':'Local ignored log/ only; GitHub contains source and textual evidence'}
(folder/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
font=FontProperties(fname=str(root/'src/community_demo/models/fonts/wqy-zenhei.ttc'))
fig,ax=plt.subplots(figsize=(8,8),layout='constrained')
floor=mpimg.imread(root/'src/community_demo/models/community_assets/materials/textures/floor.png')
ax.imshow(floor,extent=(-2.1,2.1,-2.1,2.1),origin='upper',cmap='gray',vmin=0,vmax=1)
for d,c in zip(reports,['#f59e0b','#0ea5e9','#22c55e','#a855f7','#e11d48','#14b8a6']):
    xy=[r['pose'][:2] for r in d['records']]
    label=f"{d['scenario']} / lap {d['lap_id']}：{d['physical_distance_m']:.2f} m"
    ax.plot([p[0] for p in xy],[p[1] for p in xy],color=c,lw=1.6,alpha=.85,label=label)
ax.scatter([1.7],[1.7],s=130,marker='*',c='#a7f3d0',edgecolors='black',zorder=8)
ax.plot([.3,.3],[1.4,2.0],color='#ef4444',lw=3);ax.plot([-.4,.2],[-.51,-.51],color='#ef4444',lw=3)
ax.set(xlim=(-2.1,2.1),ylim=(-2.1,2.1),xlabel='X (m)',ylabel='Y (m)',aspect='equal')
ax.set_title('巡航优化验收：六圈实际物理轨迹',fontproperties=font,fontsize=17,pad=14)
ax.legend(prop=font,loc='lower left',framealpha=.95,fontsize=9)
out=root/'log/patrol-optimization/acceptance-routes.png';fig.savefig(out,dpi=160)
print(json.dumps({k:summary[k] for k in ['passed','completed_laps','normal_laps','controlled_fault_laps','all_edges_green','road_violation_samples','checked_event_count','confirmed_observations']},ensure_ascii=False))
print(out)
raise SystemExit(0 if summary['passed'] else 1)
