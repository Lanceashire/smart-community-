#!/usr/bin/env python3
"""Read-only live UI proof: OCR actual image frame number and inspect plate detail."""
import argparse,base64,json,re,time,urllib.request
from pathlib import Path
import cv2,numpy as np
from rapidocr import RapidOCR
cv2.setNumThreads(2)
def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--timeout',type=float,default=300);a=p.parse_args()
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
    deadline=time.monotonic()+a.timeout;payload=None;error=None
    try:
        while time.monotonic()<deadline:
            try:
                state=json.load(urllib.request.urlopen('http://127.0.0.1:8765/state',timeout=2))
                if state.get('detail'):
                    payload=json.load(urllib.request.urlopen('http://127.0.0.1:8765/snapshot',timeout=2))
                    if payload.get('report',{}).get('detail'):break
            except (OSError,ValueError):pass
            time.sleep(.3)
        else:raise RuntimeError('No live plate snapshot before timeout')
        report=payload['report'];jpeg=base64.b64decode(payload['image']);image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
        detail_bytes=base64.b64decode(payload['detail_image']);detail=cv2.imdecode(np.frombuffer(detail_bytes,np.uint8),cv2.IMREAD_COLOR)
        ocr=RapidOCR(params={'Global.model_root_dir':'src/community_demo/models/ocr','EngineConfig.onnxruntime.intra_op_num_threads':2,'EngineConfig.onnxruntime.inter_op_num_threads':2})
        read=ocr(image[:32,:180],use_det=False,use_cls=False);texts=list(read.txts) if read and read.txts else []
        match=re.search(r'Frame\s*(\d+)',' '.join(texts),re.I);frame=int(match.group(1)) if match else None
        metadata=report['detail'];checks={'native_camera_size':image.shape[:2]==(960,1280),
            'actual_rendered_frame_matches_report':frame==report['frame_id'],
            'detail_same_source_stamp':metadata['stamp']==report['stamp'],
            'detail_is_four_times_original_crop':detail is not None and detail.shape[1]==4*metadata['source_size'][0] and detail.shape[0]==4*metadata['source_size'][1],
            'events_do_not_come_from_future_frame':all(e['frame_id']<=frame for e in report['events'])}
        out.with_suffix('.jpg').write_bytes(jpeg);out.with_name('plate-detail.jpg').write_bytes(detail_bytes)
        result={'passed':all(checks.values()),'checks':checks,'frame_id':frame,'stamp':report['stamp'],'detail':metadata,'header_ocr':texts,'image':str(out.with_suffix('.jpg'))}
    except Exception as exc:result={'passed':False,'error':str(exc)}
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False),flush=True)
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
