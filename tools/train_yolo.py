#!/usr/bin/env python3
"""Train the detector on captured Gazebo frames; use an installed GPU venv."""
import argparse,json,hashlib,shutil
from pathlib import Path
from ultralytics import YOLO
import torch
p=argparse.ArgumentParser();p.add_argument('--epochs',type=int,default=45);p.add_argument('--imgsz',type=int,default=960);p.add_argument('--batch',type=int,default=6);p.add_argument('--weights',default='yolo11n.pt');p.add_argument('--name',default='community-yolo11n');a=p.parse_args()
root=Path(__file__).resolve().parents[1]
if not torch.cuda.is_available():raise RuntimeError('GPU required; do not silently train on CPU')
model=YOLO(str(root/a.weights) if (root/a.weights).exists() else a.weights)
model.train(data=str(root/'data/community/dataset.yaml'),epochs=a.epochs,imgsz=a.imgsz,batch=a.batch,device=0,workers=2,project=str(root/'log/training'),name=a.name,exist_ok=True,seed=42,patience=15,amp=True,cache=False,close_mosaic=8,fliplr=0.,flipud=0.,degrees=5.,translate=.08,scale=.2,mosaic=.5,mixup=0.,hsv_h=.015,hsv_s=.35,hsv_v=.35)
best=root/'log/training'/a.name/'weights/best.pt';destination=root/'src/community_demo/models/perception';destination.mkdir(exist_ok=True);shutil.copy2(best,destination/'best.pt')
trained=YOLO(str(best));metrics=trained.val(data=str(root/'data/community/dataset.yaml'),split='test',imgsz=a.imgsz,device=0,project=str(root/'log/training'),name='heldout-test',exist_ok=True)
report={'model':'YOLO11n','torch':torch.__version__,'ultralytics':__import__('ultralytics').__version__,'gpu':torch.cuda.get_device_name(0),'epochs_requested':a.epochs,'imgsz':a.imgsz,'batch':a.batch,'sha256':hashlib.sha256(best.read_bytes()).hexdigest(),'test_metrics':metrics.results_dict,'dataset':'Gazebo camera frames, separate seeded pose capture batches; offline geometry projection annotations require human audit','limitation':'fixed supplied 18 standee identities; simulation-only test, not real-world generalization'}
(root/'validation/yolo-training.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
