#!/usr/bin/env python3
"""Evaluate local and Hugging Face checkpoints with identical images and IoU rules."""
import argparse
import hashlib
import json
from pathlib import Path

from ultralytics import YOLO
from ultralytics.models.yolo.detect.val import DetectionValidator
from ultralytics.utils.metrics import ConfusionMatrix
from torchvision.ops import batched_nms

ROOT=Path(__file__).resolve().parents[1]

def dataset_fingerprint():
    return hashlib.sha256((ROOT/'validation/comparison-dataset.json').read_bytes()).hexdigest()

class MappedValidator(DetectionValidator):
    mapping={}
    def init_metrics(self,model):
        super().init_metrics(model)
        self.names=self.data['names'];self.nc=len(self.names)
        self.metrics.names=self.names;self.confusion_matrix=ConfusionMatrix(names=self.names)
        self.class_map=list(range(1,self.nc+1))

    def postprocess(self,predictions):
        outputs=super().postprocess(predictions)
        for output in outputs:
            cls=output['cls'].long();keep=cls.new_zeros(cls.shape,dtype=__import__('torch').bool)
            mapped=cls.clone()
            for old,new in self.mapping.items():
                mask=cls==old;keep|=mask;mapped[mask]=new
            for key in output:output[key]=output[key][keep]
            output['cls']=mapped[keep].float()
            if len(set(self.mapping.values()))<len(self.mapping) and len(output['cls']):
                # After collapsing identities/arrows, apply the same class-wise NMS
                # a native one-class/three-class detector uses.
                retain=batched_nms(output['bboxes'],output['conf'],output['cls'],self.args.iou)
                for key in output:output[key]=output[key][retain]
        return outputs

def evaluate(name,weights,group,mapping,split):
    model=YOLO(str(weights))
    MappedValidator.mapping=mapping
    metrics=model.val(validator=MappedValidator,data=str(ROOT/'data/comparison_prepared'/group/'dataset.yaml'),
        split=split,imgsz=960,batch=8,device=0,workers=2,conf=.001,iou=.7,max_det=300,
        project=str(ROOT/'log/comparison'),name=f'{name}-{group}-{split}',exist_ok=True,plots=False,verbose=False)
    classes={metrics.names[int(cls)]:dict(zip(['precision','recall','mAP50','mAP50-95'],map(float,metrics.box.class_result(i)))) for i,cls in enumerate(metrics.box.ap_class_index)}
    report={'model':name,'group':group,'split':split,'sha256':hashlib.sha256(weights.read_bytes()).hexdigest(),
        'dataset_manifest_sha256':dataset_fingerprint(),'class_mapping':mapping,'merged_class_nms':len(set(mapping.values()))<len(mapping),
        'metrics':metrics.results_dict,'per_class':classes,'speed_ms':metrics.speed}
    path=ROOT/'validation/comparison-results';path.mkdir(exist_ok=True)
    (path/f'{name}-{group}-{split}.json').write_text(json.dumps(report,indent=2))
    print('RESULT',json.dumps(report),flush=True)
    return report

def main():
    p=argparse.ArgumentParser();p.add_argument('--include-s',action='store_true');p.add_argument('--splits',nargs='+',default=['val','test']);a=p.parse_args()
    local=ROOT/'src/community_demo/models/perception/best.pt';hf=ROOT/'log/hf-models'
    variants=[('current-yolo11n',local,{'full':{i:i for i in range(22)},'person':{i:0 for i in range(18)},'plate':{18:0},'lights':{19:0,20:1,21:2}}),
        ('hf-yolo11s-coco',hf/'yolo11s.pt',{'person':{0:0}}),
        ('hf-plate-yolo11n',hf/'license-plate-finetune-v1n.pt',{'plate':{0:0}}),
        ('hf-plate-yolo11s',hf/'license-plate-finetune-v1s.pt',{'plate':{0:0}}),
        ('hf-traffic-yolov8',hf/'best_roboflow.pt',{'lights':{14:0,20:1,7:2}})]
    if (hf/'lisa-yolo26l.pt').exists():
        variants.append(('hf-lisa-yolo26l',hf/'lisa-yolo26l.pt',{'lights':{0:2,1:2,2:2,3:0,4:0,5:1,6:1}}))
    if a.include_s:
        variants.append(('finetuned-yolo11s',ROOT/'src/community_demo/models/perception/yolo11s-comparison.pt',{'full':{i:i for i in range(22)},'person':{i:0 for i in range(18)},'plate':{18:0},'lights':{19:0,20:1,21:2}}))
    reports=[]
    for name,path,groups in variants:
        for group,mapping in groups.items():
            for split in a.splits:
                output=ROOT/'validation/comparison-results'/f'{name}-{group}-{split}.json'
                cached=json.loads(output.read_text()) if output.exists() else None
                merge=len(set(mapping.values()))<len(mapping)
                reports.append(cached if cached and cached.get('sha256')==hashlib.sha256(path.read_bytes()).hexdigest() and cached.get('dataset_manifest_sha256')==dataset_fingerprint() and bool(cached.get('merged_class_nms',False))==merge else evaluate(name,path,group,mapping,split))
    (ROOT/'validation/hf-model-comparison.json').write_text(json.dumps({'input_size':960,'protocol':'Shared frozen splits, subset metrics only compare corresponding classes; COCO person model cannot identify standees', 'reports':reports},indent=2))

if __name__=='__main__':main()
