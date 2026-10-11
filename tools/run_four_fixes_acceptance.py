#!/usr/bin/env python3
"""Own one world at a time; preserve actual failures and every source hash."""
import argparse,hashlib,json,math,os,signal,subprocess,time
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
F=ROOT/'validation/four-fixes-2026-10-04'
LOG=ROOT/'log/four-fixes'

def stop(process):
    if process.poll() is not None:return
    os.killpg(process.pid,signal.SIGINT)
    try:process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=8)

def start_in_phase(phase,delay):
    # This is the evaluator's launch timing, never a control input.
    import rclpy
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('acceptance_cold_start_phase');state={'phase':'unknown','seen_red':False}
    def received(m):
        state['phase']=m.data
        if m.data=='red':state['seen_red']=True
    node.create_subscription(String,'/simulation/traffic_light_state',received,10)
    try:
        for retry in range(5):
            result=subprocess.run(['gz','service','-s','/world/community/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: false'],capture_output=True)
            if result.returncode==0:break
        else:raise RuntimeError('Cold-start world did not unpause')
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if state['seen_red'] and state['phase']==phase:break
        else:raise RuntimeError('Cold-start phase not observed')
        started=time.monotonic()
        while time.monotonic()-started<delay:rclpy.spin_once(node,timeout_sec=.05)
        if state['phase']!=phase:raise RuntimeError('Cold-start requested phase expired')
        return {'requested_phase':phase,'actual_phase_before_patrol_launch':state['phase'],'delay_wall_s':delay,'source':'Evaluator observes real lamps; patrol receives camera colours only'}
    finally:node.destroy_node();rclpy.try_shutdown()

def main():
    global F,LOG
    parser=argparse.ArgumentParser();parser.add_argument('--cases',default=None);parser.add_argument('--extended',action='store_true');parser.add_argument('--output',default='validation/four-fixes-2026-10-04');parser.add_argument('--log-dir',default='log/four-fixes');a=parser.parse_args()
    F=ROOT/a.output;LOG=ROOT/a.log_dir
    F.mkdir(parents=True,exist_ok=True);LOG.mkdir(parents=True,exist_ok=True)
    if a.cases is None:a.cases='main,start-plus,start-minus,sensor-stress,stress-seed-17,stress-seed-91,cold-green,cold-yellow' if a.extended else 'main,start-plus,start-minus,sensor-stress'
    manifest=[]
    for name,dx,dy,da in [('start-plus',.025,-.025,.035),('start-minus',-.025,.025,-.035)]:
        tree=ET.parse(ROOT/'src/community_demo/worlds/community.sdf')
        robot=tree.getroot().find("world/model[@name='patrol_bot']")
        robot.find('pose').text=f'{1.7+dx} {1.7+dy} .002 0 0 {math.pi+da}'
        world=ROOT/'src/community_demo/worlds'/f'four-fixes-{name}.sdf'
        tree.write(world,encoding='utf-8',xml_declaration=True)
        manifest.append({'name':name,'world':world.name,'dx_m':dx,'dy_m':dy,'yaw_delta_rad':da,'sha256':hashlib.sha256(world.read_bytes()).hexdigest()})
    (F/'start-matrix.json').write_text(json.dumps({'variants':manifest,'method':'Change initial spawn only; nominal localization prior unchanged; no teleport while driving'},indent=2)+'\n')
    files=['src/community_demo/community_demo/'+n+'.py' for n in ['patrol','perception','localization','pose_sync','person_evidence','signal_gate','traffic_signals','sensor_stress']]+['src/community_demo/launch/demo.launch.py','src/community_demo/worlds/community.sdf','src/community_demo/models/perception/yolo11s-comparison.pt','src/community_demo/maps/roads.pgm','src/community_demo/maps/planning-obstacles.json','src/community_demo/config/navigation.yaml','tools/audit_patrol_cycles.py','tools/verify_event_images.py','tools/run_four_fixes_acceptance.py','tools/run_complete_acceptance.py','tools/summarize_four_fixes.py','tools/summarize_optimization.py','tools/check_optimization_controls.py','tools/build_navigation_map.py','tools/launch_full.sh','src/community_signal_system/src/SignalSystem.cc','install/community_signal_system/lib/libcommunity_signal_system.so']
    files=sorted(set(files+[str(p.relative_to(ROOT)) for base in ('src/community_demo/community_demo','src/community_demo/config','src/community_demo/maps','src/community_demo/launch','src/sentry_motion_manager','tools') for p in (ROOT/base).rglob('*') if p.is_file() and p.suffix in ('.py','.yaml','.json','.sh','.cpp','.hpp','.h','.xml')]+['install/sentry_motion_manager/lib/sentry_motion_manager/sentry_motion_manager_node','validation/optimization-2026-10-05/strict-spec.json','src/community_demo/worlds/layout.json','src/community_demo/models/community_assets/materials/textures/floor.png']))
    hashes={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files}
    (F/'runtime-hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    env=os.environ.copy();env['FASTDDS_BUILTIN_TRANSPORTS']='UDPv4';env['PYTHONPATH']=str(ROOT/'.deps')+os.pathsep+env.get('PYTHONPATH','')
    results=[]
    for name in a.cases.split(','):
        world=f'four-fixes-{name}.sdf' if name.startswith('start-') else 'community.sdf'
        output=F/name;output.mkdir(exist_ok=True)
        with (LOG/(name+'-runtime.log')).open('w') as runtime:
            command=['bash','tools/launch_full.sh','gui:=false','paused:=true','world:='+str(ROOT/'src/community_demo/worlds'/world)]
            if name=='sensor-stress' or name.startswith('stress-seed-'):
                seed=int(name.split('-')[-1]) if name.startswith('stress-seed-') else 20261004
                command+=['stress:=true','stress_seed:='+str(seed)]
            if name.startswith('cold-'):command.append('patrol:=false')
            process=subprocess.Popen(command,cwd=ROOT,env=env,stdout=runtime,stderr=subprocess.STDOUT,start_new_session=True)
            extra=None
            try:
                if name.startswith('cold-'):
                    phase=name.split('-')[1];info=start_in_phase(phase,2. if phase=='green' else 0.)
                    (output/'cold-start.json').write_text(json.dumps(info,indent=2)+'\n')
                    extra=subprocess.Popen(['ros2','run','community_demo','patrol','--ros-args','-p','use_sim_time:=true'],cwd=ROOT,env=env,stdout=runtime,stderr=subprocess.STDOUT,start_new_session=True)
                audit=['python3','tools/audit_patrol_cycles.py','--laps',('4' if a.extended else '3') if name=='main' else '1','--fault-lap','0','--unpause','--timeout','350','--output',str(output.relative_to(ROOT))]
                # Preserve the comparable +2/+6 cases, then repeat a provably
                # sufficient early green. +10 reached a late phase and did not
                # supply the second already-green instance required by scoring.
                if name=='main':audit+=['--restart-phases','green,green','--green-delays','2,6,0' if a.extended else '2,6']
                print('CASE_START',name,flush=True)
                with (LOG/(name+'-audit.log')).open('w') as log:
                    result=subprocess.run(audit,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1250 if name=='main' else 470)
                (output/'terminal-events.log').write_text((ROOT/'log/perception-terminal.log').read_text())
                results.append({'case':name,'exit_code':result.returncode,'passed':result.returncode==0})
                print('CASE_FINISHED',json.dumps(results[-1]),flush=True)
                if result.returncode:raise RuntimeError(name+' failed; preserve evidence and repair before rerun')
            finally:
                if extra is not None:stop(extra)
                stop(process)
        time.sleep(2)
    assert hashes=={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files},'Runtime source changed during suite'
    (F/'suite-processes.json').write_text(json.dumps({'passed':all(r['passed'] for r in results),'cases':results,'source_hashes_unchanged':True},indent=2)+'\n')
    return 0
if __name__=='__main__':raise SystemExit(main())
