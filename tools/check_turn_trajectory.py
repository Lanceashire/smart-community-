#!/usr/bin/env python3
"""Read-only smoothness audit of actual lower-junction motion; no truth control."""
import argparse,json,math
from pathlib import Path


def metrics(path):
    lap=json.loads(Path(path).read_text())
    events=lap['status_changes']
    start=next(x['wall_s'] for x in events if x['status'].startswith('REACHED lower_south'))
    end=next(x['wall_s'] for x in events if x['status'].startswith('REACHED bottom_east'))
    samples=[r for r in lap['records'] if start<=r['wall_s']<=end]
    variation=0.;directions=[];last=None
    for r in samples:
        if last and r['speed']>.025 and last['speed']>.025 and 0<r['t']-last['t']<=.2:
            delta=math.atan2(math.sin(r['pose'][2]-last['pose'][2]),math.cos(r['pose'][2]-last['pose'][2]))
            variation+=abs(delta)
            rate=delta/(r['t']-last['t'])
            sign=1 if rate>.1 else (-1 if rate<-.1 else 0)
            if sign and (not directions or sign!=directions[-1]):directions.append(sign)
        last=r
    recoveries=[x for x in events if start<=x['wall_s']<=end and x['status'].startswith('RECOVER_ROAD')]
    pivot_samples=[r for r in samples if r['status'].startswith('ALIGN_ROUTE')]
    return {'run_id':lap['run_id'],'lap_passed':lap['passed'],
            'start_wall_s':start,'end_wall_s':end,'duration_wall_s':end-start,
            'moving_heading_variation_deg':math.degrees(variation),
            'moving_turn_direction_changes':max(0,len(directions)-1),
            'road_recovery_episodes':len(recoveries),
            'alignment_linear_command_max_m_s':max((abs(r['command'][0]) for r in pivot_samples),default=None),
            'road_violations':lap['road_violation_samples'],'red_straddles':lap['red_straddle_samples'],
            'minimum_post_clearance_m':lap['minimum_post_clearance_m'],
            'whole_lap_completed_sim_s':next(x['t'] for x in lap['records'] if x['status'].startswith('COMPLETE'))-lap['records'][0]['t'],
            'samples':len(samples)}


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--baseline',required=True);p.add_argument('--output',required=True);p.add_argument('--plot');a=p.parse_args()
    run=Path(a.run);cases=sorted(run.glob('*/lap-001.json'));assert cases,'No physical lap results'
    baseline=metrics(a.baseline);rows={path.parent.name:metrics(path) for path in cases};checks={}
    for name,row in rows.items():
        checks.update({name+'_complete':row['lap_passed'],name+'_no_road_recovery':row['road_recovery_episodes']==0,
            name+'_moving_heading_variation_below_35_deg':row['moving_heading_variation_deg']<=35.,
            name+'_moving_direction_changes_at_most_two':row['moving_turn_direction_changes']<=2,
            name+'_alignment_commands_are_stationary':row['alignment_linear_command_max_m_s']==0.,
            name+'_no_road_or_red_violation':row['road_violations']==0 and row['red_straddles']==0,
            name+'_post_clearance_at_least_5cm':row['minimum_post_clearance_m']>=.05})
    normal=rows.get('mapped-navigation');checks['normal_reduces_moving_heading_variation_by_half']=bool(normal) and normal['moving_heading_variation_deg']<=baseline['moving_heading_variation_deg']*.5
    result={'passed':all(checks.values()),'checks':checks,'failed_checks':[k for k,v in checks.items() if not v],
            'baseline_file':str(Path(a.baseline).resolve()),'baseline':baseline,'cases':rows,
            'definition':'Gazebo source-stamped yaw variation between REACHED lower_south and REACHED bottom_east, counting adjacent samples only while actual speed >2.5cm/s. Turn direction uses 0.1rad/s deadband. Alignment is evaluated separately with zero linear command.',
            'scope':'Additional internal smoothness gates, not official scoring thresholds. Safety, full-body light state and perception checks remain in the independent lap audit.'}
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.font_manager import FontProperties
        font=FontProperties(fname=str(Path(__file__).resolve().parents[1]/'src/community_demo/models/fonts/wqy-zenhei.ttc'))
        plt.rcParams['font.family']=font.get_name()
        fig,ax=plt.subplots(figsize=(10,5),layout='constrained')
        for file,label,colour in [(Path(a.baseline),'修复前实际轨迹','#f59e0b'),(run/'mapped-navigation/lap-001.json','修复后实际轨迹','#079b95')]:
            data=json.loads(file.read_text());m=metrics(file);r=[x for x in data['records'] if m['start_wall_s']<=x['wall_s']<=m['end_wall_s']]
            ax.plot([x['pose'][0] for x in r],[x['pose'][1] for x in r],label=label,color=colour,lw=2)
            for x in r[::12]:
                px,py,z=x['pose'];ax.arrow(px,py,.025*math.cos(z),.025*math.sin(z),width=.0006,head_width=.007,color=colour,alpha=.6,length_includes_head=True)
        ax.axhline(-2,color='#555555',lw=2);ax.axhline(-1.4,color='#555555',lw=2)
        ax.add_patch(plt.Rectangle((.195,-1.6325),.025,.025,color='#9d2929'))
        ax.text(.2075,-1.59,'下方灯柱',ha='center',fontproperties=font)
        ax.scatter([1.1],[-1.7],marker='x',s=70,color='#079b95')
        ax.set(xlim=(-.2,1.2),ylim=(-2.03,-1.37),xlabel='X / m',ylabel='Y / m',aspect='equal')
        ax.set_title('第二个红绿灯后的路段：真实物理轨迹比较',fontproperties=font)
        ax.legend(prop=font,loc='upper left');ax.grid(alpha=.2)
        fig.savefig(a.plot,dpi=170);plt.close(fig)
    print(json.dumps({'passed':result['passed'],'failed':result['failed_checks'],'baseline':baseline,'cases':rows},ensure_ascii=False,indent=2))
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
