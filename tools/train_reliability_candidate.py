#!/usr/bin/env python3
"""Train-only weak-class oversampling; keep frozen validation and test unchanged."""
import hashlib,json
from pathlib import Path
import torch,yaml
from ultralytics import YOLO
ROOT=Path(__file__).resolve().parents[1]
base=ROOT/'data/comparison_prepared/full'
out=ROOT/'data/reliability_candidate';out.mkdir(parents=True,exist_ok=True)
for kind in ['images','labels']:(out/kind/'train').mkdir(parents=True,exist_ok=True)
counts={'original':0,'extra':0};weak={13,16,18}
for image in sorted((base/'images/train').glob('*')):
    label=base/'labels/train'/image.with_suffix('.txt').name
    classes={int(line.split()[0]) for line in label.read_text().splitlines() if line.strip()}
    for n in range(3 if classes & weak else 1):
        stem=f'{image.stem}_repeat{n}'
        for source,target in [(image,out/'images/train'/(stem+image.suffix)),(label,out/'labels/train'/(stem+'.txt'))]:
            if not target.exists():target.symlink_to(source.resolve())
        counts['original' if n==0 else 'extra']+=1
data=yaml.safe_load((base/'dataset.yaml').read_text())
data.update(path=str(out),train='images/train',val=str(base/'images/val'),test=str(base/'images/test'))
(out/'dataset.yaml').write_text(yaml.safe_dump(data,allow_unicode=True))
assert torch.cuda.is_available()
model=YOLO(str(ROOT/'src/community_demo/models/perception/yolo11s-comparison.pt'))
model.train(data=str(out/'dataset.yaml'),epochs=12,imgsz=960,batch=4,device=0,workers=2,
            project=str(ROOT/'log/training'),name='reliability-2026-10-04',exist_ok=False,
            optimizer='AdamW',lr0=.0003,lrf=.1,seed=104,patience=5,amp=True,
            fliplr=0.,flipud=0.,degrees=4.,translate=.06,scale=.15,mosaic=.2,
            close_mosaic=4,hsv_s=.3,hsv_v=.3,cache=False)
best=ROOT/'log/training/reliability-2026-10-04/weights/best.pt'
report={'weights':str(best.relative_to(ROOT)),'sha256':hashlib.sha256(best.read_bytes()).hexdigest(),
        'train_sampling':counts,'weak_class_ids':sorted(weak),'epochs_requested':12,
        'validation_test_unchanged':True,'new_unique_training_images':0,
        'scope':'Training images are repeated only to adjust weak-class sampling; no validation/test training.'}
(ROOT/'validation/reliability-2026-10-04/training.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
