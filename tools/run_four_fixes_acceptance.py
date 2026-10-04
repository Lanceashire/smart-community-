#!/usr/bin/env python3
"""Own one world at a time; preserve actual failures and every source hash."""
import argparse,hashlib,json,math,os,signal,subprocess,time
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
F=ROOT/'validation/four-fixes-2026-10-04'
LOG=ROOT/'log/four-fixes';LOG.mkdir(parents=True,exist_ok=True)

def stop(process):
    if process.poll() is not None:return
    os.killpg(process.pid,signal.SIGINT)
    try:process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=8)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--cases',default='main,start-plus,start-minus,sensor-stress');a=parser.parse_args()
    manifest=[]
    for name,dx,dy,da in [('start-plus',.025,-.025,.035),('start-minus',-.025,.025,-.035)]:
        tree=ET.parse(ROOT/'src/community_demo/worlds/community.sdf')
        robot=tree.getroot().find("world/model[@name='patrol_bot']")
        robot.find('pose').text=f'{1.1+dx} {1.7+dy} .002 0 0 {math.pi+da}'
        world=ROOT/'src/community_demo/worlds'/f'four-fixes-{name}.sdf'
        tree.write(world,encoding='utf-8',xml_declaration=True)
        manifest.append({'name':name,'world':world.name,'dx_m':dx,'dy_m':dy,'yaw_delta_rad':da,'sha256':hashlib.sha256(world.read_bytes()).hexdigest()})
    (F/'start-matrix.json').write_text(json.dumps({'variants':manifest,'method':'Change initial spawn only; nominal localization prior unchanged; no teleport while driving'},indent=2)+'\n')
    files=['src/community_demo/community_demo/'+n+'.py' for n in ['patrol','perception','localization','person_evidence','signal_gate','traffic_signals','sensor_stress']]+['src/community_demo/launch/demo.launch.py','src/community_demo/worlds/community.sdf','src/community_demo/models/perception/yolo11s-comparison.pt','tools/audit_patrol_cycles.py','tools/launch_full.sh','src/community_signal_system/src/SignalSystem.cc','install/community_signal_system/lib/libcommunity_signal_system.so']
    hashes={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files}
    (F/'runtime-hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    env=os.environ.copy();env['FASTDDS_BUILTIN_TRANSPORTS']='UDPv4';env['PYTHONPATH']=str(ROOT/'.deps')+os.pathsep+env.get('PYTHONPATH','')
    results=[]
    for name in a.cases.split(','):
        world=f'four-fixes-{name}.sdf' if name.startswith('start-') else 'community.sdf'
        output=F/name;output.mkdir(exist_ok=True)
        with (LOG/(name+'-runtime.log')).open('w') as runtime:
            command=['bash','tools/launch_full.sh','gui:=false','paused:=true','world:='+str(ROOT/'src/community_demo/worlds'/world)]
            if name=='sensor-stress':command.append('stress:=true')
            process=subprocess.Popen(command,cwd=ROOT,env=env,stdout=runtime,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                audit=['python3','tools/audit_patrol_cycles.py','--laps','3' if name=='main' else '1','--fault-lap','0','--unpause','--timeout','350','--output',str(output.relative_to(ROOT))]
                if name=='main':audit+=['--restart-phases','green,green','--green-delays','2,6']
                print('CASE_START',name,flush=True)
                with (LOG/(name+'-audit.log')).open('w') as log:
                    result=subprocess.run(audit,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1250 if name=='main' else 470)
                (output/'terminal-events.log').write_text((ROOT/'log/perception-terminal.log').read_text())
                results.append({'case':name,'exit_code':result.returncode,'passed':result.returncode==0})
                print('CASE_FINISHED',json.dumps(results[-1]),flush=True)
                if result.returncode:raise RuntimeError(name+' failed; preserve evidence and repair before rerun')
            finally:stop(process)
        time.sleep(2)
    assert hashes=={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files},'Runtime source changed during suite'
    (F/'suite-processes.json').write_text(json.dumps({'passed':all(r['passed'] for r in results),'cases':results,'source_hashes_unchanged':True},indent=2)+'\n')
    return 0
if __name__=='__main__':raise SystemExit(main())
