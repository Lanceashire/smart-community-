#!/usr/bin/env python3
"""Independently read the rendered frame number from saved event images."""
import argparse,json,re,hashlib,os,time
from pathlib import Path
# Bound the auxiliary OCR worker's CPU pools, independently of the robot.
os.environ['OMP_NUM_THREADS']='2'
os.environ['OPENBLAS_NUM_THREADS']='2'
import cv2
from rapidocr import RapidOCR
cv2.setNumThreads(2)
started=time.monotonic()

root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--report',required=True);p.add_argument('--output',required=True);a=p.parse_args()
d=json.loads((root/a.report).read_text());events=d['perception']['events']
ocr=RapidOCR(params={'Global.model_root_dir':str(root/'src/community_demo/models/ocr'),'EngineConfig.onnxruntime.intra_op_num_threads':2,'EngineConfig.onnxruntime.inter_op_num_threads':2})
cache={};checks=[]
for e in events:
 path=root/'log/perception'/e['image'];digest=hashlib.sha256(path.read_bytes()).hexdigest()
 if digest not in cache:
  image=cv2.imread(str(path))
  # Frame number is drawn at the left of the top banner. Read that actual
  # prefix instead of recognizing unrelated community-count text across 1280px.
  # The banner is already a known single line. Direct OCR avoids an expensive
  # text-region detector and still reads digits from the original image pixels.
  item_started=time.monotonic();read=ocr(image[:32,:180],use_det=False,use_cls=False)
  texts=list(read.txts) if read and read.txts else []
  match=re.search(r'Frame\s*(\d+)',' '.join(texts),flags=re.I)
  cache[digest]={'rendered_frame_id':int(match.group(1)) if match else None,'header_ocr':texts}
  print(json.dumps({'checked_image':path.name,'ocr_wall_s':round(time.monotonic()-item_started,3),'rendered_frame_id':cache[digest]['rendered_frame_id']}),flush=True)
 seen=cache[digest]
 checks.append({'sequence':e['sequence'],'image':e['image'],'expected_frame_id':e['frame_id'],'sha256':digest,**seen,'passed':seen['rendered_frame_id']==e['frame_id']})
report={'passed':bool(checks) and all(c['passed'] for c in checks),'method':'Direct single-line OCR of actual saved JPEG Frame prefix (x 0:180, y 0:32); compare independently read digits with terminal/JSON event frame_id','unique_images':len(cache),'event_count':len(checks),'events':checks,'verification_wall_s':round(time.monotonic()-started,3),'cpu_pool_threads':2,'read_only_roi':[0,0,180,32]}
(root/a.output).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='events'}));raise SystemExit(0 if report['passed'] else 1)
