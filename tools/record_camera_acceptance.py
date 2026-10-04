#!/usr/bin/env python3
"""Record wall-time live camera JPEGs; never synthesize robot motion or labels."""
import argparse,json,subprocess,time,urllib.request
import numpy as np
from pathlib import Path
import cv2

def main():
    p=argparse.ArgumentParser();p.add_argument('--lap',type=int,required=True);p.add_argument('--output',required=True);a=p.parse_args();root=Path(__file__).resolve().parents[1];out=root/a.output;out.parent.mkdir(parents=True,exist_ok=True)
    video=None;start=None;finish=None;frames=0;next_frame=0;events=[];run_id=None;last_status=None;deadline=time.monotonic()+800
    try:
        while time.monotonic()<deadline:
            now=time.monotonic()
            try:d=json.loads((root/'log/perception/report.json').read_text())
            except (OSError,ValueError):time.sleep(.05);continue
            if start is None and d.get('lap_id')==a.lap and not d.get('status','').startswith('COMPLETE'):
                run_id=d['run_id'];start=now;next_frame=now
                video=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','bgr24','-s','960x720','-r','5','-i','pipe:0','-an','-c:v','libx264','-preset','veryfast','-crf','24','-threads','2','-pix_fmt','yuv420p','-movflags','+faststart','-y',str(out)],stdin=subprocess.PIPE)
                print('CAMERA_RECORD_STARTED',run_id,a.lap,flush=True)
            if start is not None:
                if (d.get('run_id'),d.get('lap_id'))!=(run_id,a.lap):raise RuntimeError('Lap changed during recording')
                if d.get('status')!=last_status:
                    last_status=d.get('status');events.append({'video_wall_s':now-start,'sim_s':d['stamp'],'status':last_status})
                if now>=next_frame:
                    # HTTP returns one immutable JPEG buffer, never a half-written disk file.
                    with urllib.request.urlopen('http://127.0.0.1:8765/frame.jpg',timeout=3) as response:
                        pixels=response.read()
                    frame=cv2.imdecode(np.frombuffer(pixels,np.uint8),cv2.IMREAD_COLOR)
                    if frame is not None:video.stdin.write(cv2.resize(frame,(960,720)).tobytes());frames+=1
                    next_frame+=.2
                if d.get('status','').startswith('COMPLETE'):
                    if finish is None:finish=now
                    if now-finish>2:break
            time.sleep(.02)
        else:raise RuntimeError('Camera recording timeout')
    finally:
        if video is not None:video.stdin.close();video.wait(timeout=30)
    report={'passed':video is not None and video.returncode==0 and finish is not None,'run_id':run_id,'lap_id':a.lap,'frames':frames,'fps':5,'duration_s':frames/5,'source':'Actual immutable /frame.jpg HTTP camera buffer; wall-time recording, resized to 960x720','events':events,'file':str(out.relative_to(root))};out.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False));return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
