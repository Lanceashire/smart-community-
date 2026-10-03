#!/usr/bin/env python3
"""Run two worlds whose only difference is initial spawn pose; own subprocess groups."""
import json,os,signal,subprocess,time
from pathlib import Path

root=Path(__file__).resolve().parents[1]
folder=root/'validation/performance-2026-10-02'
log=root/'log/performance';log.mkdir(parents=True,exist_ok=True)
env=os.environ.copy();env['AIC_PERCEPTION_MODEL']='src/community_demo/models/perception/yolo11s-comparison.pt'
env['PYTHONPATH']=str(root/'.deps')+os.pathsep+env.get('PYTHONPATH','')

def stop_owned_launch():
    targets=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:words=[s.decode() for s in (p/'cmdline').read_bytes().split(b'\0') if s]
        except (OSError,UnicodeError):continue
        if len(words)>4 and words[1]=='/opt/ros/jazzy/bin/ros2' and words[2:5]==['launch','community_demo','demo.launch.py']:targets.append(int(p.name))
    if len(targets)!=1:raise RuntimeError(f'Expected one owned community launch, got {targets}')
    os.kill(targets[0],signal.SIGINT)
    deadline=time.monotonic()+20
    while Path(f'/proc/{targets[0]}').exists() and time.monotonic()<deadline:time.sleep(.2)
    if Path(f'/proc/{targets[0]}').exists():raise RuntimeError('Previous launch did not stop')
    time.sleep(2)

def main():
    # Main audit writes summary only after all event-image checks have completed.
    deadline=time.monotonic()+1600
    while not (folder/'candidate/summary.json').is_file():
        if (folder/'candidate/failure.json').is_file():raise RuntimeError('Main audit failed')
        if time.monotonic()>deadline:raise RuntimeError('Main audit completion timed out')
        time.sleep(2)
    if not json.loads((folder/'candidate/summary.json').read_text())['passed']:raise RuntimeError('Main audit has failed checks')
    (folder/'candidate/terminal-events.log').write_text((root/'log/perception-terminal.log').read_text())
    stop_owned_launch()
    manifest=json.loads((root/'validation/patrol-optimization-2026-10-02/start-variants.json').read_text())
    for index,variant in enumerate(manifest['variants']):
        name=variant['name'];destination=folder/name;destination.mkdir(parents=True,exist_ok=True)
        runtime=open(log/f'{name}-runtime.log','w')
        process=subprocess.Popen(['bash','tools/launch_full.sh','gui:=false','paused:=true','world:='+variant['world']],cwd=root,env=env,stdout=runtime,stderr=subprocess.STDOUT,start_new_session=True)
        probe=None
        try:
            if index==0:
                probe=subprocess.Popen(['python3','tools/probe_obstacle_recovery.py','--output',str(folder/'obstacle-recovery.json')],cwd=root,env=env,stdout=runtime,stderr=subprocess.STDOUT)
            with open(log/f'{name}-audit.log','w') as output:
                result=subprocess.run(['python3','tools/audit_patrol_cycles.py','--laps','1','--fault-lap','0','--unpause','--timeout','350','--output',str(destination.relative_to(root))],cwd=root,env=env,stdout=output,stderr=subprocess.STDOUT,timeout=470)
            (destination/'terminal-events.log').write_text((root/'log/perception-terminal.log').read_text())
            print('START_VARIANT_RESULT',name,result.returncode,flush=True)
            if probe and probe.wait(timeout=20):raise RuntimeError('Physical obstacle recovery failed')
            if result.returncode:raise RuntimeError(f'{name} failed; evidence retained')
            if index==len(manifest['variants'])-1:
                (folder/'active-runtime.json').write_text(json.dumps({'pid':process.pid,'world':variant['world'],'state':'Completed and stopped; GUI retained'},indent=2)+'\n')
                process=None  # Last completed world remains visible for user acceptance.
        finally:
            if probe and probe.poll() is None:probe.terminate();probe.wait(timeout=10)
            if process:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=8)
            runtime.close()
        time.sleep(2)
    (folder/'start-matrix.json').write_text(json.dumps({'passed':True,'variants':manifest['variants'],'method':'Different initial spawn only; nominal localization prior unchanged; no teleports during patrol'},indent=2)+'\n')
    return 0
if __name__=='__main__':raise SystemExit(main())
