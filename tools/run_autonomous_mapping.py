#!/usr/bin/env python3
"""Fresh autonomous mapping, then isolated saved-map navigation and obstacle test."""
import argparse,hashlib,json,os,subprocess,time
from pathlib import Path
from run_simulation_refinement import stop
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--cases',default='autonomous-mapping,mapped-navigation,mapped-dynamic');p.add_argument('--saved-map');a=p.parse_args();out=ROOT/a.output;out.mkdir(parents=True,exist_ok=True)
 assert not (out/'pipeline.json').exists(),'Preserve previous attempt'
 def runtime_sources():
  return [p for p in (ROOT/'src').rglob('*') if p.is_file() and '__pycache__' not in str(p)]+[p for p in (ROOT/'tools').iterdir() if p.is_file() and p.suffix in ('.py','.sh','.bash')]
 sources=runtime_sources()
 hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources};(out/'runtime-hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
 def frozen():return set(str(p.relative_to(ROOT)) for p in runtime_sources())==set(hashes) and all((ROOT/p).is_file() and hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
 rows=[];error=None
 try:
  for name in a.cases.split(','):
   assert frozen();case=out/name;case.mkdir(exist_ok=True);active=[];handles=[]
   def spawn(cmd,label):
    f=(case/(label+'.log')).open('w');handles.append(f);q=subprocess.Popen(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);active.append(q);return q
   try:
    mapping=name=='autonomous-mapping';dynamic=name=='mapped-dynamic'
    observer=spawn(['python3','tools/observe_signal_integrity.py','--output',str(case/'signal-integrity.json')],'signals')
    capture=None;recorder=None;probe=None;map_check=None
    if mapping:
     spawn(['ros2','run','ros_gz_bridge','parameter_bridge','/recording/overview@sensor_msgs/msg/Image[gz.msgs.Image'],'overview-bridge')
     capture=spawn([os.environ['AIC_VISION_PYTHON'],'tools/collect_mapping_evidence.py','--output',str(case)],'mapping')
    else:
     saved=Path(a.saved_map) if a.saved_map else out/'autonomous-mapping/navigation.yaml'
     assert saved.is_file(),'Build and verify the fresh map first'
     map_check=spawn([os.environ['AIC_VISION_PYTHON'],'tools/check_mapped_navigation.py','--map',str(saved),'--output',str(case)],'map-check')
     spawn(['ros2','run','ros_gz_bridge','parameter_bridge','/recording/overview@sensor_msgs/msg/Image[gz.msgs.Image'],'overview-bridge')
     recorder=spawn([os.environ['AIC_VISION_PYTHON'],'tools/record_patrol_video.py','--output',str((case/'patrol.mp4').relative_to(ROOT)), '--ready',str((case/'recorder-ready.json').relative_to(ROOT))],'recording')
     deadline=time.monotonic()+30
     while not (case/'recorder-ready.json').exists() and time.monotonic()<deadline:
      if recorder.poll() is not None:raise RuntimeError('Recorder startup failed')
      time.sleep(.1)
     assert (case/'recorder-ready.json').exists()
     if dynamic:probe=spawn(['python3','tools/probe_dynamic_interference.py','--output',str(case/'dynamic-interference.json')],'dynamic')
    extra=[] if mapping else ['slam:=false','navigation_map:='+str(saved),'navigation_config:='+str(ROOT/'src/community_demo/config/navigation_mapped.yaml')]
    spawn(['bash','tools/launch_full.sh','gui:=false','paused:=true','world:='+('community.sdf' if mapping else 'community_recording.sdf'),*extra],'runtime')
    audit=spawn(['python3','tools/audit_patrol_cycles.py','--laps','1','--fault-lap','0','--unpause','--timeout','360','--output',str(case.relative_to(ROOT))],'audit')
    print('CASE_STARTED',name,flush=True);rc=audit.wait(timeout=520)
    auxiliary=True
    for q in (capture,recorder,probe,map_check):
     if q is not None:auxiliary=q.wait(timeout=30)==0 and auxiliary
    for q in reversed(active):stop(q)
    s=json.loads((case/'summary.json').read_text()) if (case/'summary.json').exists() else {'passed':False}
    lamps=json.loads((case/'signal-integrity.json').read_text())
    row={'case':name,'passed':rc==0 and s['passed'] and lamps['passed'] and auxiliary and frozen(),'audit_exit':rc,'auxiliary_passed':auxiliary,'source_frozen':frozen()};rows.append(row);print('CASE_FINISHED',json.dumps(row),flush=True)
    if not row['passed']:raise RuntimeError('Acceptance failed: '+name)
   finally:
    for q in reversed(active):stop(q)
    for f in handles:f.close()
 except Exception as exc:error=str(exc)
 report={'passed':len(rows)==len(a.cases.split(',')) and all(x['passed'] for x in rows) and frozen(),'cases':rows,'error':error,'source_frozen':frozen()}
 (out/'pipeline.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True);return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
