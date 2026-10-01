#!/usr/bin/env python3
"""Authorized YOLO11s experiment. Never overwrite the deployed nano model."""
import argparse
import hashlib
import json
from pathlib import Path
import torch
from ultralytics import YOLO

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser();p.add_argument('--epochs',type=int,default=45);a=p.parse_args()
    if not torch.cuda.is_available():raise RuntimeError('GPU required')
    model=YOLO(str(ROOT/'log/hf-models/yolo11s.pt'))
    model.train(data=str(ROOT/'data/comparison_prepared/full/dataset.yaml'),epochs=a.epochs,imgsz=960,
        batch=4,device=0,workers=2,project=str(ROOT/'log/training'),name='hf-yolo11s-comparison',
        exist_ok=False,seed=42,patience=15,amp=True,cache=False,close_mosaic=8,fliplr=0.,flipud=0.,
        degrees=5.,translate=.08,scale=.2,mosaic=.5,mixup=0.,hsv_h=.015,hsv_s=.35,hsv_v=.35)
    best=ROOT/'log/training/hf-yolo11s-comparison/weights/best.pt'
    (ROOT/'validation/yolo11s-training.json').write_text(json.dumps({'model':'YOLO11s','initial_weights':'Ultralytics/YOLO11 on Hugging Face',
        'initial_revision':'8b8ac7d1fae7468f85dbf89670dd66f41f485aab','epochs_requested':a.epochs,'batch':4,'imgsz':960,
        'sha256':hashlib.sha256(best.read_bytes()).hexdigest(),'training_images':736,
        'validation':'Frozen expanded, deduplicated comparison set','limitation':'Current nano had 45+25 epochs with an earlier validation set; this measures usable checkpoint performance, not a controlled architecture-only ablation'},indent=2))

if __name__=='__main__':main()
