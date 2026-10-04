#!/usr/bin/env python3
"""Fail before ROS launch when required files, GPU inference or OCR are unusable."""
import argparse,json,sys,time
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('--model',required=True);p.add_argument('--output',default='log/preflight.json');a=p.parse_args()
    root=Path(__file__).resolve().parents[1];checks={};errors=[];details={}
    for file in [a.model,'src/community_demo/maps/roads.pgm','src/community_demo/maps/roads.yaml','src/community_demo/maps/planning-obstacles.json','install/setup.bash',
                 'install/community_signal_system/lib/libcommunity_signal_system.so']:
        checks['file:'+file]=(root/file).is_file()
    try:
        import torch,numpy as np
        from ultralytics import YOLO
        from rapidocr import RapidOCR
        checks['cuda_available']=torch.cuda.is_available()
        if not checks['cuda_available']:raise RuntimeError('NVIDIA CUDA GPU unavailable')
        details.update(python=sys.version.split()[0],torch=torch.__version__,gpu=torch.cuda.get_device_name(0))
        model=YOLO(str(root/a.model));result=model.predict(np.zeros((960,1280,3),np.uint8),imgsz=960,device=0,verbose=False)[0]
        checks['gpu_inference']=True;checks['identity_classes']=len(result.names)==22
        ocr=RapidOCR(params={'Global.model_root_dir':str(root/'src/community_demo/models/ocr')})
        ocr(np.full((48,160,3),255,np.uint8),use_det=False,use_cls=False)
        checks['ocr_inference']=True
    except Exception as e:errors.append(str(e))
    report={'passed':bool(checks) and all(checks.values()) and not errors,'checks':checks,'errors':errors,'details':details,
            'time_unix':time.time(),'scope':'Static files, actual GPU/YOLO and OCR inference; live sensors remain controller safety gates'}
    path=root/a.output;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report));return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
