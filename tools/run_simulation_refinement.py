#!/usr/bin/env python3
"""Run isolated, frozen-source laps and preserve every candidate attempt."""
import argparse,hashlib,json,os,signal,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def stop(p):
    if p is None or p.poll() is not None:return
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid,signal.SIGTERM)
        try:p.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    parser.add_argument('--cases',default='normal-performance,recorded-normal,dynamic-obstacle,moving-faults,sensor-stress')
    a=parser.parse_args();out=ROOT/a.output;out.mkdir(parents=True,exist_ok=True)
    assert not (out/'pipeline.json').exists(),'Preserve the previous attempt'
    def runtime_sources():
        return [p for p in (ROOT/'src').rglob('*') if p.is_file() and '__pycache__' not in str(p)]+[p for p in (ROOT/'tools').iterdir() if p.is_file() and p.suffix in ('.py','.sh','.bash')]
    sources=runtime_sources()
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    (out/'runtime-hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    def frozen():return (set(str(p.relative_to(ROOT)) for p in runtime_sources())==set(hashes) and
                        all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==v for p,v in hashes.items()))
    rows=[];error=None;started=time.time()
    try:
        for name in a.cases.split(','):
            assert frozen();case=out/name;case.mkdir(exist_ok=True);active=[];handles=[]
            def spawn(cmd,label):
                f=(case/(label+'.log')).open('w');handles.append(f)
                p=subprocess.Popen(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);active.append(p);return p
            try:
                observer=spawn(['python3','tools/observe_signal_integrity.py','--output',str(case/'signal-integrity.json')],'signals')
                film=name=='recorded-normal';dynamic=name=='dynamic-obstacle'
                probe=spawn(['python3','tools/probe_dynamic_interference.py','--output',str(case/'dynamic-interference.json')],'dynamic') if dynamic else None
                recorder=None
                snapshot_check=None
                if film:
                    spawn(['ros2','run','ros_gz_bridge','parameter_bridge','/recording/overview@sensor_msgs/msg/Image[gz.msgs.Image'],'overview-bridge')
                    recorder=spawn([os.environ['AIC_VISION_PYTHON'],'tools/record_patrol_video.py','--output',str((case/'patrol.mp4').relative_to(ROOT)),
                                    '--ready',str((case/'recorder-ready.json').relative_to(ROOT))],'recording')
                    deadline=time.monotonic()+30
                    while not (case/'recorder-ready.json').exists() and time.monotonic()<deadline:
                        if recorder.poll() is not None:raise RuntimeError('Recorder failed at startup')
                        time.sleep(.1)
                    assert (case/'recorder-ready.json').exists()
                    snapshot_check=spawn([os.environ['AIC_VISION_PYTHON'],'tools/check_live_snapshot.py','--output',str(case/'live-snapshot.json')],'live-snapshot')
                extra=['world:=community_recording.sdf'] if film else []
                if name=='dim-light':extra=['world:=community_dim.sdf']
                if name=='sensor-stress':extra+=['stress:=true','stress_seed:=20261004']
                spawn(['bash','tools/launch_full.sh','gui:=false','paused:=true',*extra],'runtime')
                cmd=['python3','tools/audit_patrol_cycles.py','--laps','1','--fault-lap','1' if name=='moving-faults' else '0',
                     '--unpause','--timeout','360','--output',str(case.relative_to(ROOT))]
                if name=='moving-faults':cmd+=['--moving-faults']
                audit=spawn(cmd,'audit');print('CASE_STARTED',name,flush=True)
                rc=audit.wait(timeout=520)
                auxiliary=True
                if recorder:auxiliary=recorder.wait(timeout=25)==0
                if snapshot_check:auxiliary=snapshot_check.wait(timeout=25)==0 and auxiliary
                if probe:auxiliary=probe.wait(timeout=25)==0 and auxiliary
                for p in reversed(active):stop(p)
                summary_path=case/'summary.json'
                summary=json.loads(summary_path.read_text()) if summary_path.exists() else {'passed':False}
                lamps=json.loads((case/'signal-integrity.json').read_text())
                row={'case':name,'audit_exit':rc,'passed':rc==0 and summary['passed'] and lamps['passed'] and auxiliary,
                     'signals':lamps['passed'],'auxiliary_passed':auxiliary,'source_frozen':frozen()}
                rows.append(row);print('CASE_FINISHED',json.dumps(row),flush=True)
                if not row['passed']:
                    failure_path=case/'failure.json'
                    failure=json.loads(failure_path.read_text()) if failure_path.exists() else {}
                    row['failure']=failure.get('error') or failure.get('perception',{}).get('status') or 'Auxiliary or integrity check failed'
                    raise RuntimeError('Acceptance failed: '+name+'; '+row['failure'])
            finally:
                for p in reversed(active):stop(p)
                for f in handles:f.close()
    except Exception as exc:error=str(exc)
    report={'passed':len(rows)==len(a.cases.split(',')) and all(r['passed'] for r in rows) and frozen(),
            'cases':rows,'source_frozen':frozen(),'error':error,'started_unix':started,'finished_unix':time.time(),
            'scope':'Actual Gazebo laps; existing competition compliance checks unchanged; moving obstacles audit stop/wait/resume, not unrestricted bypass. No YOLO retraining.'}
    (out/'pipeline.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
    return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
