#!/usr/bin/env python3
"""Run the existing-container acceptance pipeline and preserve every exit code."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--extended',action='store_true');a=p.parse_args()
    f=ROOT/a.output;f.mkdir(parents=True,exist_ok=True);log=ROOT/'log'/f.name;log.mkdir(parents=True,exist_ok=True)
    vision=os.environ['AIC_VISION_PYTHON'];steps=[];observer=None;started=time.time();passed=False
    def run(name,cmd,expected=0):
        begin=time.monotonic();print('STEP_START',name,flush=True)
        with (log/(name+'.log')).open('w') as out:r=subprocess.run(cmd,cwd=ROOT,stdout=out,stderr=subprocess.STDOUT)
        steps.append({'step':name,'exit_code':r.returncode,'expected_exit':expected,'wall_s':round(time.monotonic()-begin,2)})
        print('STEP_FINISHED',json.dumps(steps[-1]),flush=True)
        if r.returncode!=expected:raise RuntimeError(name+' failed; preserve logs')
    try:
        for tool,report in [('check_four_fixes','control-regression'),('check_signal_handoff','callback-regression'),('check_reliability','association-regression')]:
            run(tool,['python3','tools/'+tool+'.py','--output',str(f/report)+'.json'])
            assert json.loads((f/(report+'.json')).read_text())['passed']
        if a.extended:run('optimization-regression',[vision,'tools/check_optimization_controls.py','--output',str(f/'optimization-regression.json')])
        assert json.loads((f/'world-preservation.json').read_text())['passed']
        run('preflight',[vision,'tools/preflight.py','--model','src/community_demo/models/perception/yolo11s-comparison.pt','--output',str(f/'preflight.json')])
        run('negative-image-counterexample',[vision,'tools/verify_event_images.py','--report',str(f/'negative-input.json'),'--output',str(f/'image-negative-counterexample.json')],expected=1)
        negative=json.loads((f/'image-negative-counterexample.json').read_text());assert not negative['passed'] and negative['events'][0]['rendered_frame_id']==3
        with (log/'signal-observer.log').open('w') as out:
            observer=subprocess.Popen(['python3','tools/observe_signal_integrity.py','--output',str(f/'signal-integrity.json')],cwd=ROOT,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
            try:run('live-matrix',['python3','tools/run_four_fixes_acceptance.py','--output',str(f.relative_to(ROOT)),'--log-dir',str(log.relative_to(ROOT))]+(['--extended'] if a.extended else []))
            finally:
                if observer.poll() is None:os.killpg(observer.pid,signal.SIGINT)
                observer.wait(timeout=20)
                steps.append({'step':'independent-signal-observer','exit_code':observer.returncode,'expected_exit':0})
                if observer.returncode:raise RuntimeError('Signal observer failed')
        run('strict-summary',['python3','tools/summarize_four_fixes.py','--output',str(f.relative_to(ROOT))]+(['--main-laps','4'] if a.extended else []))
        if a.extended:run('optimization-summary',['python3','tools/summarize_optimization.py','--output',str(f.relative_to(ROOT))])
        passed=True;return 0
    finally:
        report={'passed':passed,'existing_container':'aic-development','container_rebuilt':False,'started_unix_s':started,'finished_unix_s':time.time(),'steps':steps,'scope':('Eleven live laps, three noise seeds, cold green/yellow starts and strict before/after checks' if a.extended else 'Six live laps')+' plus regressions, GPU/OCR/plugin checks, actual signal poses and saved image OCR; no offline recovery used'}
        (f/'pipeline.json').write_text(json.dumps(report,indent=2)+'\n');print('PIPELINE',json.dumps(report),flush=True)
if __name__=='__main__':raise SystemExit(main())
