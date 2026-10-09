#!/usr/bin/env python3
"""Frozen-runtime physical acceptance and actual camera recording of the fixes."""
import hashlib,json,os,signal,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];F=ROOT/'validation/boundary-fixes-2026-10-06'
LOG=ROOT/'log/boundary-fixes-2026-10-06';LOG.mkdir(parents=True,exist_ok=True)
assert not (F/'pipeline.json').exists(), 'Preserve the previous attempt before rerunning'
files=list(json.loads((ROOT/'validation/optimization-2026-10-06/runtime-hashes.json').read_text()))
files+=['tools/check_boundary_safety.py','tools/prepare_recording_world.py','tools/record_patrol_video.py',
        'tools/run_boundary_fixes_acceptance.py','src/community_demo/worlds/community_recording.sdf',
        'src/community_demo/package.xml']
hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(set(files))}
(F/'runtime-hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
def frozen():return all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==v for p,v in hashes.items())
def stop(p):
    if p is None or p.poll() is not None:return
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid,signal.SIGTERM)
        try:p.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
start=time.time();steps=[];active=[]
try:
    for name,extra,fault,film in [('recorded-normal',['world:=community_recording.sdf'],0,True),
                                   ('node-faults',[],1,False),('sensor-stress',['stress:=true','stress_seed:=20261004'],0,False),
                                   ('normal-performance',[],0,False)]:
        assert frozen();case=F/name;case.mkdir(parents=True,exist_ok=True);active=[]
        handles=[]
        try:
            def spawn(cmd,log):
                f=(LOG/log).open('w');handles.append(f)
                p=subprocess.Popen(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
                active.append(p);return p
            observer=spawn(['python3','tools/observe_signal_integrity.py','--output',str(case/'signal-integrity.json')],name+'-observer.log')
            bridge=None;rec=None
            if film:
                bridge=spawn(['ros2','run','ros_gz_bridge','parameter_bridge','/recording/overview@sensor_msgs/msg/Image[gz.msgs.Image'],name+'-bridge.log')
                rec=spawn([os.environ['AIC_VISION_PYTHON'],'tools/record_patrol_video.py','--lap','1',
                           '--output','log/boundary-fixes-2026-10-06/patrol-full-lap.mp4',
                           '--ready',str((case/'recorder-ready.json').relative_to(ROOT))],name+'-record.log')
                deadline=time.monotonic()+30
                while not (case/'recorder-ready.json').exists() and time.monotonic()<deadline:
                    if rec.poll() is not None:raise RuntimeError('recorder startup failed')
                    time.sleep(.1)
                assert (case/'recorder-ready.json').exists()
            world=spawn(['bash','tools/launch_full.sh','gui:=false','paused:=true',*extra],name+'-runtime.log')
            print('ACCEPTANCE_CASE_STARTED',name,flush=True)
            auditlog=(LOG/(name+'-audit.log')).open('w');handles.append(auditlog)
            audit=subprocess.run(['python3','tools/audit_patrol_cycles.py','--laps','1','--fault-lap',str(fault),
                    '--unpause','--timeout','360','--output',str(case.relative_to(ROOT))],cwd=ROOT,
                    stdout=auditlog,stderr=subprocess.STDOUT,timeout=540)
            recorded_ok=True
            if rec is not None:recorded_ok=rec.wait(timeout=25)==0
            stop(world);stop(observer);stop(bridge)
            summary=json.loads((case/'summary.json').read_text())
            lamps=json.loads((case/'signal-integrity.json').read_text())
            row={'case':name,'audit_exit':audit.returncode,'passed':audit.returncode==0 and summary['passed'] and lamps['passed'] and recorded_ok,
                 'signals':lamps['passed'],'recorded_video_passed':recorded_ok if film else None,'source_frozen':frozen()}
            steps.append(row);print('ACCEPTANCE_CASE_FINISHED',json.dumps(row),flush=True)
            if not row['passed']:raise RuntimeError('physical acceptance failed: '+name)
        finally:
            for p in reversed(active):stop(p)
            active=[]
            for f in handles:f.close()
finally:
    for p in reversed(active):stop(p)
    report={'passed':len(steps)==4 and all(x['passed'] and x['source_frozen'] for x in steps) and frozen(),
            'cases':steps,'source_frozen':frozen(),'started_unix':start,'finished_unix':time.time(),
            'scope':'Four actual Gazebo laps: full normal lap recorded, three node pauses, sensor noise/dropout, and unrecorded normal speed confirmation; unchanged acceptance thresholds. No model retraining.'}
    (F/'pipeline.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
