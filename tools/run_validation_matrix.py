#!/usr/bin/env python3
"""Capture six scene/light batches with separate validation and test seeds."""
import json
import os
import signal
import subprocess
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def complete(batch,count):
    p=ROOT/'data/comparison'/(batch+'-manifest.json')
    return p.exists() and len(json.loads(p.read_text())['frames'])==count

def main():
    for li,layout in enumerate(['training','community']):
        for ci,condition in enumerate(['normal','dim','bright']):
            batches=[]
            for split,base,count in [('val',1201,100 if li==0 else 60),('test',3201,30)]:
                seed=base+li*100+ci*10
                batch=f'expanded-{layout}-{condition}-{split}-{seed}'
                if not complete(batch,count):batches.append((split,seed,batch,count))
            if not batches:continue
            world=ROOT/'log/validation-worlds'/f'{layout}-{condition}.sdf'
            with (ROOT/'log'/f'capture-{layout}-{condition}.log').open('w') as log:
                launch=subprocess.Popen(['ros2','launch',str(ROOT/'src/community_demo/launch/validation_scene.launch.py'),f'world:={world}'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    for split,seed,batch,count in batches:
                        print('CAPTURE_START',batch,flush=True)
                        path=ROOT/'log'/(batch+'.log')
                        with path.open('w') as capture_log:
                            subprocess.run(['python3',str(ROOT/'tools/expand_validation.py'),'--batch',batch,'--split',split,'--count',str(count),'--seed',str(seed),'--condition',condition,'--layout',('training-layout.json' if li==0 else 'layout.json')],stdout=capture_log,stderr=subprocess.STDOUT,check=True)
                        print('CAPTURE_FINISHED',batch,count,flush=True)
                finally:
                    if launch.poll() is None:
                        os.killpg(launch.pid,signal.SIGINT)
                        try:launch.wait(timeout=12)
                        except subprocess.TimeoutExpired:
                            os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=5)
    print('VALIDATION_MATRIX_DONE',flush=True)

if __name__=='__main__':main()
