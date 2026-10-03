#!/usr/bin/env python3
"""Aggregate final-code audits and create local-only performance figures."""
import hashlib,json,statistics
from pathlib import Path
from summarize_performance import summarize
root=Path(__file__).resolve().parents[1];folder=root/'validation/performance-2026-10-02'
base=summarize(folder/'baseline/lap-001.json')
final=[]
for group in ['candidate','start-plus','start-minus']:
    for p in sorted((folder/group).glob('lap-*.json')):
        if p.name.endswith('-images.json'):continue
        d=summarize(p);d['scenario']=group;d['source']=str(p.relative_to(root))
        if group=='start-plus':d['mode']='initial offset plus controlled physical obstacle'
        final.append(d)
assert len(final)==5, f'Expected five final-code audits; found {len(final)}'
obstacle=json.loads((folder/'obstacle-recovery.json').read_text())
regression=json.loads((folder/'controller-regression.json').read_text())
old=json.loads((root/'validation/patrol-optimization-2026-10-02/summary.json').read_text())
unchanged={p:hashlib.sha256((root/p).read_bytes()).hexdigest()==h for p,h in old['unchanged_inputs'].items()}
paths=['src/community_demo/community_demo/patrol.py','src/community_demo/community_demo/localization.py','src/community_demo/community_demo/perception.py','src/community_demo/launch/demo.launch.py','tools/launch_full.sh','tools/audit_patrol_cycles.py','tools/check_performance_controls.py','tools/probe_obstacle_recovery.py','tools/audit_performance_variants.py','tools/build_navigation_map.py','src/community_demo/community_demo/road_rules.py','src/community_demo/maps/roads.pgm','src/community_demo/maps/planning-obstacles.json']
sha={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in paths}
first=final[0]
comparison={k:{'baseline':base[k],'optimized':first[k],'change_percent':round((first[k]/base[k]-1)*100,2)} for k in ['completion_sim_s','completion_wall_s','position_error_max_m','position_error_p95_m','position_error_rmse_m']}
result={'passed':all(d['passed'] for d in final) and obstacle['passed'] and regression['passed'] and all(unchanged.values()),'final_laps':5,'normal_laps':3,'node_fault_laps':1,'physical_obstacle_laps':1,'baseline':base,'comparison_nominal_start':comparison,'laps':final,'minimum_post_clearance_m':min(d['minimum_post_clearance_m'] for d in final),'physical_obstacle':obstacle,'control_regression':regression,'unchanged_weights_and_world':unchanged,'source_sha256':sha,'limits':['One baseline and one nominal-start candidate provide the direct comparison; phase/offset/fault runs are listed separately','YOLO weights and offline test metrics unchanged; no claim of improved mAP','Physical obstacle recovery waits for clearance and replans the permitted route; no dynamic bypass','Local encoder/paint/wheel localization, not arbitrary-pose global relocalization','True pose is audit-only; runtime receives wheel odometry, camera, lidar and camera joint encoder','Images remain local under log/performance']}
(folder/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'passed':result['passed'],'comparison':comparison,'laps':[(d['scenario'],d['lap_id'],d['completion_sim_s'],d['position_error_max_m']) for d in final]},indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained');colors=['#64748b','#0d9488']
axs[0,0].bar(['Before','After'],[base['completion_sim_s'],first['completion_sim_s']],color=colors)
for i,d in enumerate([base,first]):axs[0,0].text(i,d['completion_sim_s']+2,f"{d['completion_sim_s']:.1f}s",ha='center')
axs[0,0].set(title='Same nominal start: completion time',ylabel='Simulation seconds',ylim=(0,max(base['completion_sim_s'],first['completion_sim_s'])*1.2))
import numpy as np
x=np.arange(3);width=.35
for i,d in enumerate([base,first]):axs[0,1].bar(x+(i-.5)*width,[100*d[k] for k in ['position_error_max_m','position_error_p95_m','position_error_rmse_m']],width,label=['Before','After'][i],color=colors[i])
axs[0,1].set(xticks=x,xticklabels=['Maximum','95th percentile','RMSE'],ylabel='Position error (cm)',title='Localization: audit against true pose');axs[0,1].legend()
for i,d in enumerate([base,first]):
    v=d['state_sim_s'];parts=[v.get('GO',0),sum(v.get(k,0) for k in ['OBSERVE','RETRY_OBSERVE']),v.get('STOP',0)];parts.append(d['completion_sim_s']-sum(parts));bottom=0
    for part,name,col in zip(parts,['Travel','Observe','Signals','Other'],['#2563eb','#f59e0b','#ef4444','#94a3b8']):
        axs[1,0].bar(i,part,bottom=bottom,color=col,label=name if i==0 else None);bottom+=part
axs[1,0].set(xticks=[0,1],xticklabels=['Before','After'],ylabel='Simulation seconds',title='Measured time by controller state');axs[1,0].legend(fontsize=8)
ax=axs[1,1];floor=root/'src/community_demo/models/community_assets/materials/textures/floor.png';ax.imshow(mpimg.imread(floor),extent=[-2.1,2.1,-2.1,2.1])
for index,d in enumerate(final):
    raw=json.loads((root/d['source']).read_text());xy=np.array([r['pose'][:2] for r in raw['records']]);ax.plot(xy[:,0],xy[:,1],lw=1.2,alpha=.8,label=f"{d['scenario']} lap {d['lap_id']}")
for post in json.loads((root/'src/community_demo/maps/planning-obstacles.json').read_text())['posts']:ax.plot(post['x'],post['y'],'rx',ms=5)
ax.set(title='Five physical patrol trajectories',xlabel='x (m)',ylabel='y (m)',aspect='equal');ax.legend(fontsize=6,loc='upper left')
fig.suptitle('AIC fixed-field performance acceptance | YOLO11s weights unchanged',fontsize=14)
out=root/'log/performance/acceptance-performance.png';fig.savefig(out,dpi=160);plt.close(fig);print(out)
