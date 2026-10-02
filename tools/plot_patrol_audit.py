#!/usr/bin/env python3
"""Plot measured trajectories on the supplied map; output remains local."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import matplotlib.image as mpimg

root=Path(__file__).resolve().parents[1]
reports=[json.loads(p.read_text()) for p in sorted((root/'validation/patrol-cycle-review/final').glob('lap-[0-9][0-9][0-9].json'))]
assert len(reports)==2 and all(r['passed'] for r in reports)
font=FontProperties(fname=str(root/'src/community_demo/models/fonts/wqy-zenhei.ttc'))
fig,ax=plt.subplots(figsize=(7.8,7.7),layout='constrained');fig.patch.set_facecolor('#f7f9fc')
background=mpimg.imread(root/'src/community_demo/models/community_assets/materials/textures/floor.png')
ax.imshow(background,extent=(-2.1,2.1,-2.1,2.1),origin='upper',cmap='gray',vmin=0,vmax=1)
for report,colour in zip(reports,['#f59e0b','#22d3ee']):
 xy=[r['pose'][:2] for r in report['records']]
 label=('故障恢复测试' if report['mode']=='fault injection' else '正常重复巡航')+f"：{report['physical_distance_m']:.2f} m"
 ax.plot([p[0] for p in xy],[p[1] for p in xy],color=colour,lw=1.9,alpha=.9,label=label)
ax.scatter([1.1],[1.7],s=120,marker='*',c='#a7f3d0',edgecolors='black',zorder=5)
ax.text(1.0,1.98,'起终点',fontproperties=font,color='#064e3b',ha='center',bbox={'facecolor':'white','alpha':.85,'edgecolor':'none'})
ax.plot([.3,.3],[1.4,2.0],color='#ef4444',lw=3)
ax.plot([-.4,.2],[-.51,-.51],color='#ef4444',lw=3)
ax.text(.4,1.35,'停止线 1',fontproperties=font,color='#ef4444')
ax.text(.24,-.6,'停止线 2',fontproperties=font,color='#ef4444')
ax.set(xlim=(-2.1,2.1),ylim=(-2.1,2.1),xlabel='X (m)',ylabel='Y (m)',aspect='equal')
ax.set_title('巡航复查：Gazebo 实际物理轨迹',fontproperties=font,fontsize=17,pad=16)
ax.legend(prop=font,loc='lower left',framealpha=.95)
out=root/'log/patrol-cycle-review/route-review.png';out.parent.mkdir(parents=True,exist_ok=True);fig.savefig(out,dpi=150);print(out)
